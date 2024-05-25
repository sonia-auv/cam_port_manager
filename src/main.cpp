#include <rclcpp/rclcpp.hpp>
#include "cam_port_manager/CaptureNode.hpp"


int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto capture_node = std::make_shared<cam_port_manager::CaptureNode>();
    capture_node->InitCameras();
    rclcpp::spin(capture_node);
    
    rclcpp::shutdown();

    return 0;
}
