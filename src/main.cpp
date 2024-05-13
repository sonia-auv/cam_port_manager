#include <rclcpp/rclcpp.hpp>
#include "cam_port_manager/CaptureNode.hpp"


int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    using rclcpp::executors::MultiThreadedExecutor;
    MultiThreadedExecutor executor;
    auto capture_node = std::make_shared<cam_port_manager::CaptureNode>();
    executor.add_node(capture_node);

    executor.spin();
    rclcpp::spin(capture_node);
    
    rclcpp::shutdown();

    return 0;
}
