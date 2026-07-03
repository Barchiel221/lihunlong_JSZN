#include <rclcpp/rclcpp.hpp>
#include "uart_to_stm32.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<uart_to_stm32::UartToStm32>();

    if (!node->initialize()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to initialize UartToStm32");
        rclcpp::shutdown();
        return -1;
    }

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
