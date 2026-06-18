#include "weapon_docking/weapon_docking_node.hpp"

namespace weapon_docking {

WeaponDockingNode::WeaponDockingNode(const rclcpp::NodeOptions & options)
: Node("weapon_docking_node", options), enable_ui_(false)
{
    using namespace std::placeholders;

    const int camera_index = this->declare_parameter<int>("camera_index", 0);
    const int frame_width = this->declare_parameter<int>("frame_width", 1280);
    const int frame_height = this->declare_parameter<int>("frame_height", 800);
    const int frame_fps = this->declare_parameter<int>("frame_fps", 120);
    const std::string frame_fourcc = this->declare_parameter<std::string>("frame_fourcc", "MJPG");
    const int buffer_size = this->declare_parameter<int>("buffer_size", 1);
    const double fx = this->declare_parameter<double>("fx", 956.80819);
    const double fy = this->declare_parameter<double>("fy", 956.46353);
    const double cx = this->declare_parameter<double>("cx", 653.72184);
    const double cy = this->declare_parameter<double>("cy", 358.43826);
    const double dist_k1 = this->declare_parameter<double>("dist_k1", -0.005408);
    const double dist_k2 = this->declare_parameter<double>("dist_k2", 0.029448);
    const double dist_p1 = this->declare_parameter<double>("dist_p1", -0.000804);
    const double dist_p2 = this->declare_parameter<double>("dist_p2", -0.00107);
    const double dist_k3 = this->declare_parameter<double>("dist_k3", 0.0);
    const double tag_size_m = this->declare_parameter<double>("tag_size_m", 0.06);
    const int target_tag_id = this->declare_parameter<int>("target_tag_id", 56);
    const int aruco_dictionary_size = this->declare_parameter<int>("aruco_dictionary_size", 1000);
    const int aruco_marker_bits = this->declare_parameter<int>("aruco_marker_bits", 6);
    const int aruco_dictionary_seed = this->declare_parameter<int>("aruco_dictionary_seed", 0);
    const double tolerance_dist = this->declare_parameter<double>("tolerance_dist", 0.01);
    control_params_.linear_x_pid = DockingPidConfig{
        this->declare_parameter<double>("linear_x_kp", 1.5),
        this->declare_parameter<double>("linear_x_ki", 0.0),
        this->declare_parameter<double>("linear_x_kd", 0.0)};
    control_params_.linear_y_pid = DockingPidConfig{
        this->declare_parameter<double>("linear_y_kp", 1.5),
        this->declare_parameter<double>("linear_y_ki", 0.0),
        this->declare_parameter<double>("linear_y_kd", 0.0)};
    control_params_.angular_pid = DockingPidConfig{
        this->declare_parameter<double>("angular_kp", 2.2),
        this->declare_parameter<double>("angular_ki", 0.0),
        this->declare_parameter<double>("angular_kd", 0.0)};
    control_params_.max_linear_speed = this->declare_parameter<double>("max_linear_speed", 0.3);
    control_params_.max_angular_speed = this->declare_parameter<double>("max_angular_speed", 0.8);
    control_params_.hold_position_tolerance =
        this->declare_parameter<double>("hold_position_tolerance", tolerance_dist);
    control_params_.hold_angle_tolerance =
        this->declare_parameter<double>("hold_angle_tolerance", tolerance_dist);
    enable_ui_ = this->declare_parameter<bool>("enable_ui", false);
    last_control_time_ = this->now();

    processor_ = std::make_unique<WeaponDockingProcessor>(
        camera_index,
        frame_width,
        frame_height,
        frame_fps,
        frame_fourcc,
        buffer_size,
        fx,
        fy,
        cx,
        cy,
        dist_k1,
        dist_k2,
        dist_p1,
        dist_p2,
        dist_k3,
        tag_size_m,
        target_tag_id,
        aruco_dictionary_size,
        aruco_marker_bits,
        aruco_dictionary_seed,
        tolerance_dist
    );

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    error_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/task/weapon_docking/error", 10);

    action_server_ = rclcpp_action::create_server<WeaponDock>(
        this, "weapon_dock",
        std::bind(&WeaponDockingNode::handle_goal, this, _1, _2),
        std::bind(&WeaponDockingNode::handle_cancel, this, _1),
        std::bind(&WeaponDockingNode::handle_accepted, this, _1));

    if (enable_ui_) {
        try {
            processor_->open_window();
            ui_timer_ = this->create_wall_timer(
                std::chrono::milliseconds(30),
                std::bind(&WeaponDockingNode::ui_update_callback, this)
            );
            RCLCPP_INFO(this->get_logger(), "图像窗口已启用。");
        } catch (const cv::Exception & e) {
            enable_ui_ = false;
            RCLCPP_WARN(
                this->get_logger(),
                "图像窗口初始化失败，已自动禁用 UI: %s",
                e.what()
            );
        }
    } else {
        RCLCPP_INFO(this->get_logger(), "图像窗口默认关闭。测试时可将参数 enable_ui 设为 true。");
    }
    
    RCLCPP_INFO(this->get_logger(), "Weapon Docking Action Server Started.");
}

rclcpp_action::GoalResponse WeaponDockingNode::handle_goal(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const WeaponDock::Goal> goal)
{
    (void)uuid;
    RCLCPP_INFO(this->get_logger(), "收到对接请求, 目标距离: %.2f m", goal->target_dist);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse WeaponDockingNode::handle_cancel(
    const std::shared_ptr<GoalHandleWeaponDock> goal_handle)
{
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "收到取消请求，停止对接");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void WeaponDockingNode::handle_accepted(const std::shared_ptr<GoalHandleWeaponDock> goal_handle)
{
    std::thread{std::bind(&WeaponDockingNode::execute_task, this, std::placeholders::_1), goal_handle}.detach();
}

void WeaponDockingNode::execute_task(const std::shared_ptr<GoalHandleWeaponDock> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "开始执行视觉解算...");
    rclcpp::Rate loop_rate(30);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<WeaponDock::Feedback>();
    auto result = std::make_shared<WeaponDock::Result>();
    reset_control();

    while (rclcpp::ok()) {
        if (goal_handle->is_canceling()) {
            publish_zero_cmd_vel();
            result->success = false;
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "任务已取消");
            return;
        }

        auto [err_x, err_y, err_z, target_depth] = processor_->compute_errors(goal->target_dist);
        
        feedback->x_error = err_x;
        feedback->y_error = err_y;
        feedback->z_error = err_z;
        goal_handle->publish_feedback(feedback);

        auto error_msg = std_msgs::msg::Float32MultiArray();
        error_msg.data = {err_x, err_y, err_z};
        error_pub_->publish(error_msg);

        const auto now = this->now();
        double control_dt = (now - last_control_time_).seconds();
        if (control_dt <= 0.0 || control_dt > 1.0) {
            control_dt = 1.0 / 30.0;
        }
        last_control_time_ = now;

        const auto has_valid_measurement = IsValidDockingMeasurement(err_x, err_y, err_z)
            && target_depth != 999.0f;
        if (!has_valid_measurement) {
            publish_zero_cmd_vel();
            ResetDockingControlState(control_state_);
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "目标 ArUco 丢失，底盘已停止。等待目标重新出现...");
            loop_rate.sleep();
            continue;
        }

        const auto target_pose = MakeDockingTargetPose(target_depth, err_y, err_z, goal->target_dist);

        auto cmd_vel = MakeZeroCmdVel();
        if (docking_stage_ == DockingStage::kVisualAlign) {
            cmd_vel = ComputeVisualAlignCmdVel(
                target_pose,
                control_params_,
                control_state_,
                control_dt);
        } else if (docking_stage_ == DockingStage::kOpenLoopX) {
            cmd_vel = ComputeXMoveCmdVel(
                target_pose,
                control_params_,
                control_state_,
                control_dt);
            if (IsDockingXWithinTolerance(target_pose, control_params_)) {
                publish_zero_cmd_vel();
                docking_stage_ = DockingStage::kDone;
            } else {
                cmd_vel_pub_->publish(cmd_vel);
            }
            loop_rate.sleep();
            continue;
        }
        cmd_vel_pub_->publish(cmd_vel);

        if (err_x != 999.0f) {
            RCLCPP_INFO(
                this->get_logger(),
                "[锁定Tag] X误差: %.1fmm, Y误差: %.1fmm, 姿态误差: %.1fmrad, 目标距离: %.1fmm",
                err_x,
                err_y,
                err_z,
                target_depth
            );
        } else {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "等待视野中出现目标 ArUco...");
        }

        docking_stage_ = has_valid_measurement
            ? NextDockingStageAfterTargetCheck(docking_stage_, target_pose, control_params_)
            : docking_stage_;

        if (docking_stage_ == DockingStage::kDone) {
            publish_zero_cmd_vel();
            RCLCPP_INFO(this->get_logger(), "对接误差已满足要求，对接成功！");
            break;
        }
        loop_rate.sleep();
    }

    publish_zero_cmd_vel();

    if (rclcpp::ok()) {
        result->success = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "任务结束。");
    }
}


void WeaponDockingNode::publish_zero_cmd_vel()
{
    if (cmd_vel_pub_) {
        cmd_vel_pub_->publish(MakeZeroCmdVel());
    }
}

void WeaponDockingNode::reset_control()
{
    ResetDockingControlState(control_state_);
    docking_stage_ = DockingStage::kVisualAlign;
    last_control_time_ = this->now();
}

void WeaponDockingNode::ui_update_callback() {
    if (!enable_ui_) {
        return;
    }

    processor_->update_preview();
    cv::Mat frame = processor_->get_latest_display_frame();
    if (!frame.empty()) {
        try {
            cv::imshow("USB Camera ArUco Docking Probe", frame);
            cv::waitKey(1);
        } catch (const cv::Exception & e) {
            enable_ui_ = false;
            ui_timer_.reset();
            RCLCPP_WARN(
                this->get_logger(),
                "图像窗口刷新失败，已自动禁用 UI: %s",
                e.what()
            );
        }
    }
}

} // namespace weapon_docking
