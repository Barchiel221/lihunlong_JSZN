#include <ros/ros.h>
#include <memory>
#include <sstream>
#include <iomanip>
#include "serial_comm/serial_comm.h"

// 简单的串口测试节点
// 打开 /dev/ttyS6, 波特率 921600，异步接收并打印收到的原始字节（十六进制）

int main(int argc, char** argv) {
	ros::init(argc, argv, "uart_test_node");
	ros::NodeHandle nh("~");

	const std::string port = "/dev/ttyS6";
	const unsigned int baud = 921600;

	auto comm = std::make_unique<serial_comm::SerialComm>();

	if (!comm->initialize(port, baud)) {
		ROS_FATAL("Failed to open serial port %s at %u: %s", port.c_str(), baud, comm->get_last_error().c_str());
		return 1;
	}

	ROS_INFO("Serial port opened: %s @ %u", port.c_str(), baud);

	// 数据回调：把收到的字节以十六进制打印
	comm->start_async_read(
		[](const std::vector<uint8_t>& data) {
			if (data.empty()) return;
			std::ostringstream oss;
			oss << std::hex << std::setfill('0');
			for (size_t i = 0; i < data.size(); ++i) {
				oss << std::setw(2) << static_cast<int>(data[i]);
				if (i + 1 < data.size()) oss << ' ';
			}
			ROS_INFO_STREAM("[UART RX] " << oss.str());
		},
		[](const std::string& err) {
			ROS_ERROR_STREAM("Serial error: " << err);
		}
	);

	// 主循环，ROS 控制退出
	ros::spin();

	ROS_INFO("Shutting down uart_test_node, closing serial port");
	// 停止异步读取并关闭串口
	comm->stop_async_read();
	comm->close();

	return 0;
}

