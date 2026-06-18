#ifndef WEAPON_DOCKING_NODE_HPP_
#define WEAPON_DOCKING_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "weapon_docking/action/weapon_dock.hpp"
#include "weapon_docking/docking_control.hpp"
#include "weapon_docking/weapon_docking_processor.hpp"
#include <memory>
#include <thread>

namespace weapon_docking {

class WeaponDockingNode : public rclcpp::Node {
public:
    using WeaponDock = weapon_docking::action::WeaponDock;
    using GoalHandleWeaponDock = rclcpp_action::ServerGoalHandle<WeaponDock>;

    explicit WeaponDockingNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~WeaponDockingNode() override = default;

private:
    // Action Server 回调
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const WeaponDock::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleWeaponDock> goal_handle);

    void handle_accepted(const std::shared_ptr<GoalHandleWeaponDock> goal_handle);

    // ★ 图1要求：多线程异步执行，防止阻塞 ROS 回调
    void execute_task(const std::shared_ptr<GoalHandleWeaponDock> goal_handle);

    void ui_update_callback();
    void publish_zero_cmd_vel();
    void reset_control();

    rclcpp::TimerBase::SharedPtr ui_timer_;
    bool enable_ui_;

    rclcpp_action::Server<WeaponDock>::SharedPtr action_server_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr error_pub_;
    std::unique_ptr<WeaponDockingProcessor> processor_;
    DockingControlParams control_params_;
    DockingControlState control_state_;
    DockingStage docking_stage_;
    rclcpp::Time last_control_time_;
};

} // namespace weapon_docking

#endif // WEAPON_DOCKING_NODE_HPP_
