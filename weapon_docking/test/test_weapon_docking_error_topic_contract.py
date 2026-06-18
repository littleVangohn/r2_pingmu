from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def test_weapon_docking_publishes_cmd_vel_and_xyz_error_topic():
    header = (ROOT / "include/weapon_docking/weapon_docking_node.hpp").read_text()
    source = (ROOT / "src/weapon_docking_node.cpp").read_text()

    assert 'rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_' in header
    assert '#include "std_msgs/msg/float32_multi_array.hpp"' in header
    assert 'rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr error_pub_' in header
    assert 'create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10)' in source
    assert 'create_publisher<std_msgs::msg::Float32MultiArray>("/task/weapon_docking/error", 10)' in source
    assert 'error_msg.data = {err_x, err_y, err_z}' in source
    assert 'error_pub_->publish(error_msg)' in source
    assert 'cmd_vel_pub_->publish(cmd_vel)' in source


def test_weapon_docking_uses_custom_aruco_6x6_1000_and_6cm_tag():
    params = (ROOT / "config/params.yaml").read_text()
    processor_header = (ROOT / "include/weapon_docking/weapon_docking_processor.hpp").read_text()
    processor_source = (ROOT / "src/weapon_docking_processor.cpp").read_text()
    cmake = (ROOT / "CMakeLists.txt").read_text()

    assert "tag_size_m: 0.06" in params
    assert "aruco_dictionary_size: 1000" in params
    assert "aruco_marker_bits: 6" in params
    assert "aruco_dictionary_seed: 0" in params
    assert "#include <opencv2/aruco.hpp>" in processor_header
    assert "cv::aruco::generateCustomDictionary" in processor_source
    assert "cv::aruco::detectMarkers" in processor_source
    assert "cv::SOLVEPNP_IPPE_SQUARE" in processor_source
    assert "apriltag" not in cmake.lower()
