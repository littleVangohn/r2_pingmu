#ifndef WEAPON_DOCKING_PROCESSOR_HPP_
#define WEAPON_DOCKING_PROCESSOR_HPP_

#include <tuple>
#include <deque>
#include <vector>
#include <optional>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <mutex>

namespace weapon_docking {

class PoseSmoother {
public:
    PoseSmoother(int history_size = 15, float ema_alpha = 0.35);
    float update(float new_value);
private:
    int history_size_;
    float alpha_;
    std::deque<float> history_;
    std::optional<float> ema_value_;
};

class WeaponDockingProcessor {
public:
    explicit WeaponDockingProcessor(
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
    );
    ~WeaponDockingProcessor();

    void update_preview();
    std::tuple<float, float, float, float> compute_errors(float target_dist);
    bool is_docking_complete(float x_err, float y_err, float z_err);

    void open_window();
    void close_window();

    cv::Mat get_latest_display_frame();

private:
    float tolerance_dist_;
    float tag_size_meters_;
    int target_tag_id_;
    int dictionary_size_;
    int marker_bits_;
    int dictionary_seed_;
    int frame_width_;
    int frame_height_;
    std::string window_name_;

    cv::VideoCapture cap_;

    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;

    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> detector_params_;
    
    PoseSmoother x_smoother_;
    PoseSmoother y_smoother_;
    PoseSmoother z_smoother_;

    bool window_is_open_;
    std::tuple<float, float, float, float> latest_measurement_;
    cv::Mat latest_display_frame_;
    std::mutex processing_mutex_;
    std::mutex state_mutex_;
};

} // namespace weapon_docking
#endif
