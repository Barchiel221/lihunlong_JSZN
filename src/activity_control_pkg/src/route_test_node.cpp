#include <clocale>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include "activity_control_pkg/route_target_publisher.hpp"

int main(int argc, char** argv)
{
    std::setlocale(LC_ALL, "");
    rclcpp::init(argc, argv);

    auto route_node =
        std::make_shared<activity_control_pkg::RouteTargetPublisherNode>();
    auto test_node =
        std::make_shared<activity_control_pkg::RouteTestNode>(route_node);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(route_node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
