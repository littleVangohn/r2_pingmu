#include "weapon_docking/weapon_docking_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    
    // 使用 MultiThreadedExecutor 以充分利用多线程优势
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<weapon_docking::WeaponDockingNode>();
    
    executor.add_node(node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}