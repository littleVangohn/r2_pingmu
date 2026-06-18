#include "weapon_docking/weapon_docking_processor.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

namespace weapon_docking {

namespace {

constexpr float kInvalidMeasurement = 999.0f;

int make_fourcc(const std::string & fourcc) {
    if (fourcc.size() != 4) {
        throw std::invalid_argument("Camera FOURCC must be exactly 4 characters.");
    }
    return cv::VideoWriter::fourcc(fourcc[0], fourcc[1], fourcc[2], fourcc[3]);
}

bool is_valid_measurement(const std::tuple<float, float, float, float> & measurement) {
    return std::get<0>(measurement) != kInvalidMeasurement
        && std::get<1>(measurement) != kInvalidMeasurement
        && std::get<2>(measurement) != kInvalidMeasurement
        && std::get<3>(measurement) != kInvalidMeasurement;
}

float compute_yaw_error_mrad(const cv::Mat & rvec) {
    cv::Mat rotation_matrix;
    cv::Rodrigues(rvec, rotation_matrix);

    const double normal_x = rotation_matrix.at<double>(0, 2);
    const double normal_z = rotation_matrix.at<double>(2, 2);
    const double yaw_error_rad = std::atan2(-normal_x, normal_z);
    return static_cast<float>(yaw_error_rad * 1000.0);
}

}  // namespace

PoseSmoother::PoseSmoother(int history_size, float ema_alpha)
: history_size_(history_size), alpha_(ema_alpha)
{
}

float PoseSmoother::update(float new_value) {
    if (history_size_ <= 1) {
        return new_value;
    }

    if (history_.size() >= static_cast<size_t>(history_size_)) {
        history_.pop_front();
    }
    history_.push_back(new_value);

    std::vector<float> sorted_history(history_.begin(), history_.end());
    std::nth_element(
        sorted_history.begin(),
        sorted_history.begin() + sorted_history.size() / 2,
        sorted_history.end()
    );
    const float robust_median = sorted_history[sorted_history.size() / 2];

    if (!ema_value_.has_value()) {
        ema_value_ = robust_median;
    } else {
        ema_value_ = alpha_ * robust_median + (1.0f - alpha_) * ema_value_.value();
    }

    return 0.5f * robust_median + 0.5f * ema_value_.value();
}

WeaponDockingProcessor::WeaponDockingProcessor(
    int camera_index,
    int frame_width,
    int frame_height,
    int frame_fps,
    const std::string & frame_fourcc,
    int buffer_size,
    double fx,
    double fy,
    double cx,
    double cy,
    double dist_k1,
    double dist_k2,
    double dist_p1,
    double dist_p2,
double dist_k3,
    double tag_size_meters,
    int target_tag_id,
    int dictionary_size,
    int marker_bits,
    int dictionary_seed,
    double tolerance_dist_m
)
: tolerance_dist_(static_cast<float>(tolerance_dist_m)),
  tag_size_meters_(static_cast<float>(tag_size_meters)),
  target_tag_id_(target_tag_id),
  dictionary_size_(dictionary_size),
  marker_bits_(marker_bits),
  dictionary_seed_(dictionary_seed),
  frame_width_(frame_width),
  frame_height_(frame_height),
  window_name_("USB Camera ArUco Docking Probe"),
  x_smoother_(15, 0.35f),
  y_smoother_(15, 0.35f),
  z_smoother_(15, 0.35f),
  window_is_open_(false),
  latest_measurement_(kInvalidMeasurement, kInvalidMeasurement, kInvalidMeasurement, kInvalidMeasurement)
{
    if (tag_size_meters_ <= 0.0f) {
        throw std::invalid_argument("tag_size_m must be positive.");
    }
    if (dictionary_size_ <= 0) {
        throw std::invalid_argument("aruco_dictionary_size must be positive.");
    }
    if (marker_bits_ <= 0) {
        throw std::invalid_argument("aruco_marker_bits must be positive.");
    }

    camera_matrix_ = cv::Mat::eye(3, 3, CV_64F);
    camera_matrix_.at<double>(0, 0) = fx;
    camera_matrix_.at<double>(1, 1) = fy;
    camera_matrix_.at<double>(0, 2) = cx;
    camera_matrix_.at<double>(1, 2) = cy;

    dist_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);
    dist_coeffs_.at<double>(0) = dist_k1;
    dist_coeffs_.at<double>(1) = dist_k2;
    dist_coeffs_.at<double>(2) = dist_p1;
    dist_coeffs_.at<double>(3) = dist_p2;
    dist_coeffs_.at<double>(4) = dist_k3;

    cap_.open(camera_index, cv::CAP_V4L2);
    if (!cap_.isOpened()) {
        cap_.open(camera_index, cv::CAP_ANY);
    }
    if (!cap_.isOpened()) {
        throw std::runtime_error("Failed to open USB camera.");
    }

    cap_.set(cv::CAP_PROP_FOURCC, static_cast<double>(make_fourcc(frame_fourcc)));
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(frame_width));
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(frame_height));
    cap_.set(cv::CAP_PROP_FPS, static_cast<double>(frame_fps));
    if (buffer_size > 0) {
        cap_.set(cv::CAP_PROP_BUFFERSIZE, static_cast<double>(buffer_size));
    }

    dictionary_ = cv::aruco::generateCustomDictionary(
        dictionary_size_,
        marker_bits_,
        dictionary_seed_);
    detector_params_ = cv::aruco::DetectorParameters::create();
    detector_params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
}

WeaponDockingProcessor::~WeaponDockingProcessor() {
    if (cap_.isOpened()) {
        cap_.release();
    }
    close_window();
}

void WeaponDockingProcessor::open_window() {
    if (!window_is_open_) {
        cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
        window_is_open_ = true;
    }
}

void WeaponDockingProcessor::close_window() {
    if (window_is_open_) {
        window_is_open_ = false;
        cv::destroyWindow(window_name_);
    }
}

cv::Mat WeaponDockingProcessor::get_latest_display_frame() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return latest_display_frame_.clone();
}

void WeaponDockingProcessor::update_preview() {
    std::lock_guard<std::mutex> processing_lock(processing_mutex_);

    cv::Mat frame;
    if (!cap_.read(frame) || frame.empty()) {
        cv::Mat fallback(frame_height_, frame_width_, CV_8UC3, cv::Scalar(20, 20, 20));
        cv::putText(
            fallback,
            "FAILED TO READ CAMERA FRAME",
            cv::Point(20, 50),
            cv::FONT_HERSHEY_SIMPLEX,
            0.8,
            cv::Scalar(0, 0, 255),
            2
        );
        std::lock_guard<std::mutex> lock(state_mutex_);
        latest_display_frame_ = fallback;
        latest_measurement_ = {
            kInvalidMeasurement,
            kInvalidMeasurement,
            kInvalidMeasurement,
            kInvalidMeasurement,
        };
        return;
    }

    cv::Mat display = frame.clone();
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    const float optical_center_x = static_cast<float>(camera_matrix_.at<double>(0, 2));
    const float optical_center_y = static_cast<float>(camera_matrix_.at<double>(1, 2));
    cv::drawMarker(
        display,
        cv::Point(static_cast<int>(std::lround(optical_center_x)), static_cast<int>(std::lround(optical_center_y))),
        cv::Scalar(255, 0, 0),
        cv::MARKER_CROSS,
        20,
        2
    );

    std::tuple<float, float, float, float> measurement {
        kInvalidMeasurement,
        kInvalidMeasurement,
        kInvalidMeasurement,
        kInvalidMeasurement,
    };

    std::vector<std::vector<cv::Point2f>> marker_corners;
    std::vector<int> marker_ids;
    cv::aruco::detectMarkers(gray, dictionary_, marker_corners, marker_ids, detector_params_);

    if (!marker_ids.empty()) {
        cv::aruco::drawDetectedMarkers(display, marker_corners, marker_ids);
    }

    for (size_t i = 0; i < marker_ids.size(); ++i) {
        if (marker_ids[i] != target_tag_id_) {
            continue;
        }

        const auto & corners = marker_corners[i];

        const float half_size = tag_size_meters_ * 0.5f;
        const std::vector<cv::Point3f> object_points = {
            cv::Point3f(-half_size,  half_size, 0.0f),
            cv::Point3f( half_size,  half_size, 0.0f),
            cv::Point3f( half_size, -half_size, 0.0f),
            cv::Point3f(-half_size, -half_size, 0.0f),
        };

        cv::Mat rvec;
        cv::Mat tvec;
        const bool success = cv::solvePnP(
            object_points,
            corners,
            camera_matrix_,
            dist_coeffs_,
            rvec,
            tvec,
            false,
            cv::SOLVEPNP_IPPE_SQUARE
        );
        if (!success || tvec.empty() || tvec.at<double>(2) <= 0.0) {
            continue;
        }

        std::vector<cv::Point> polygon;
        polygon.reserve(corners.size());
        for (const auto & corner : corners) {
            polygon.emplace_back(
                static_cast<int>(std::lround(corner.x)),
                static_cast<int>(std::lround(corner.y))
            );
        }
        cv::polylines(display, std::vector<std::vector<cv::Point>>{polygon}, true, cv::Scalar(0, 255, 0), 2);

        cv::Point2f center(0.0f, 0.0f);
        for (const auto & corner : corners) {
            center += corner;
        }
        center *= 1.0f / static_cast<float>(corners.size());
        const cv::Point tag_center(
            static_cast<int>(std::lround(center.x)),
            static_cast<int>(std::lround(center.y)));
        cv::drawMarker(display, tag_center, cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 20, 2);
        cv::line(
            display,
            cv::Point(static_cast<int>(std::lround(optical_center_x)), static_cast<int>(std::lround(optical_center_y))),
            tag_center,
            cv::Scalar(255, 255, 0),
            2
        );

        const float tag_right_mm = static_cast<float>(tvec.at<double>(0)) * 1000.0f;
        const float tag_forward_mm = static_cast<float>(tvec.at<double>(2)) * 1000.0f;
        const float raw_x_mm = tag_forward_mm;
        const float raw_y_mm = -tag_right_mm;
        const float raw_yaw_mrad = compute_yaw_error_mrad(rvec);

        const float err_x_mm = x_smoother_.update(raw_x_mm);
        const float err_y_mm = y_smoother_.update(raw_y_mm);
        const float err_yaw_mrad = z_smoother_.update(raw_yaw_mrad);

        char line[256];
        std::snprintf(line, sizeof(line), "Tag ID: %d", target_tag_id_);
        cv::putText(display, line, cv::Point(20, 50), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(255, 255, 0), 2);
        std::snprintf(line, sizeof(line), "Tag pose: X=%.1fmm Y=%.1fmm yaw=%.1fmrad", raw_x_mm, raw_y_mm, raw_yaw_mrad);
        cv::putText(display, line, cv::Point(20, 74), cv::FONT_HERSHEY_SIMPLEX, 0.50, cv::Scalar(0, 255, 255), 2);

        measurement = {
            err_x_mm,
            err_y_mm,
            err_yaw_mrad,
            tag_forward_mm,
        };
        break;
    }

    if (!is_valid_measurement(measurement)) {
        cv::putText(
            display,
            ("WAITING FOR ARUCO " + std::to_string(target_tag_id_)),
            cv::Point(20, 50),
            cv::FONT_HERSHEY_SIMPLEX,
            0.75,
            cv::Scalar(0, 255, 255),
            2
        );
    } else {
        char error_line[256];
        std::snprintf(
            error_line,
            sizeof(error_line),
            "Dock err: X=%.1fmm Y=%.1fmm yaw=%.1fmrad",
            std::get<0>(measurement),
            std::get<1>(measurement),
            std::get<2>(measurement)
        );
        cv::putText(display, error_line, cv::Point(20, display.rows - 20), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 128), 2);
    }

    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_display_frame_ = display.clone();
    latest_measurement_ = measurement;
}

std::tuple<float, float, float, float> WeaponDockingProcessor::compute_errors(float target_dist) {
    (void)target_dist;

    update_preview();

    std::lock_guard<std::mutex> lock(state_mutex_);
    if (latest_display_frame_.empty()) {
        return {
            kInvalidMeasurement,
            kInvalidMeasurement,
            kInvalidMeasurement,
            kInvalidMeasurement,
        };
    }
    return latest_measurement_;
}

bool WeaponDockingProcessor::is_docking_complete(float x_err, float y_err, float z_err) {
    return (
        std::abs(x_err) / 1000.0f < tolerance_dist_ &&
        std::abs(y_err) / 1000.0f < tolerance_dist_ &&
        std::abs(z_err) / 1000.0f < tolerance_dist_
    );
}

}  // namespace weapon_docking
