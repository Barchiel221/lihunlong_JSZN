#include "serial_comm/serial_comm.h"
#include <boost/bind.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <iostream>
#include <fstream>
#include <dirent.h>
#include <sys/stat.h>
#include <chrono>
#include <thread>

namespace serial_comm {

// 定义静态常量成员
const uint8_t ProtocolFrame::FRAME_HEADER;
const uint8_t ProtocolFrame::ADDRESS;
const size_t ProtocolFrame::MIN_FRAME_SIZE;
const size_t ProtocolFrame::MAX_DATA_SIZE;

SerialComm::SerialComm() :
    io_service_(std::make_unique<boost::asio::io_service>()),
    read_buffer_(READ_BUFFER_SIZE),
    is_async_reading_(false),
    is_running_(false),
    read_timeout_ms_(0)
{
}

SerialComm::~SerialComm() {
    close();
}

bool SerialComm::initialize(const std::string& port_name, unsigned int baud_rate) {
    SerialConfig config;
    config.port_name = port_name;
    config.baud_rate = baud_rate;
    return initialize(config);
}

bool SerialComm::initialize(const SerialConfig& config) {
    try {
        // 保存配置
        current_config_ = config;
        
        // 如果已经打开，先关闭
        if (is_open()) {
            close();
        }

        // 创建串口对象
        serial_port_ = std::make_unique<boost::asio::serial_port>(*io_service_);
        
        // 打开串口
        boost::system::error_code ec;
        serial_port_->open(config.port_name, ec);
        if (ec) {
            std::lock_guard<std::mutex> lock(error_mutex_);
            last_error_ = "Failed to open port " + config.port_name + ": " + ec.message();
            return false;
        }

        // 配置串口参数
        if (!configure_port(config)) {
            serial_port_->close();
            return false;
        }

        // 启动IO服务线程
        start_io_service();

        std::lock_guard<std::mutex> lock(error_mutex_);
        last_error_ = "";
        return true;

    } catch (const std::exception& e) {
        std::lock_guard<std::mutex> lock(error_mutex_);
        last_error_ = "Exception during initialization: " + std::string(e.what());
        return false;
    }
}

bool SerialComm::configure_port(const SerialConfig& config) {
    try {
        boost::system::error_code ec;

        // 设置波特率
        serial_port_->set_option(boost::asio::serial_port::baud_rate(config.baud_rate), ec);
        if (ec) {
            std::lock_guard<std::mutex> lock(error_mutex_);
            last_error_ = "Failed to set baud rate: " + ec.message();
            return false;
        }

        // 设置数据位
        serial_port_->set_option(boost::asio::serial_port::character_size(config.character_size), ec);
        if (ec) {
            std::lock_guard<std::mutex> lock(error_mutex_);
            last_error_ = "Failed to set character size: " + ec.message();
            return false;
        }

        // 设置校验位
        serial_port_->set_option(boost::asio::serial_port::parity(config.parity), ec);
        if (ec) {
            std::lock_guard<std::mutex> lock(error_mutex_);
            last_error_ = "Failed to set parity: " + ec.message();
            return false;
        }

        // 设置停止位
        serial_port_->set_option(boost::asio::serial_port::stop_bits(config.stop_bits), ec);
        if (ec) {
            std::lock_guard<std::mutex> lock(error_mutex_);
            last_error_ = "Failed to set stop bits: " + ec.message();
            return false;
        }

        // 设置流控制
        serial_port_->set_option(boost::asio::serial_port::flow_control(config.flow_control), ec);
        if (ec) {
            std::lock_guard<std::mutex> lock(error_mutex_);
            last_error_ = "Failed to set flow control: " + ec.message();
            return false;
        }

        return true;

    } catch (const std::exception& e) {
        std::lock_guard<std::mutex> lock(error_mutex_);
        last_error_ = "Exception during port configuration: " + std::string(e.what());
        return false;
    }
}

bool SerialComm::is_open() const {
    return serial_port_ && serial_port_->is_open();
}

void SerialComm::close() {
    stop_async_read();
    stop_io_service();
    
    if (serial_port_ && serial_port_->is_open()) {
        boost::system::error_code ec;
        serial_port_->close(ec);
    }
    serial_port_.reset();
}

int SerialComm::write(const std::vector<uint8_t>& data) {
    if (!is_open()) {
        std::lock_guard<std::mutex> lock(error_mutex_);
        last_error_ = "Port is not open";
        return -1;
    }

    try {
        boost::system::error_code ec;
        size_t bytes_written = boost::asio::write(*serial_port_, 
                                                  boost::asio::buffer(data), ec);
        if (ec) {
            std::lock_guard<std::mutex> lock(error_mutex_);
            last_error_ = "Write error: " + ec.message();
            return -1;
        }
        return static_cast<int>(bytes_written);

    } catch (const std::exception& e) {
        std::lock_guard<std::mutex> lock(error_mutex_);
        last_error_ = "Exception during write: " + std::string(e.what());
        return -1;
    }
}

int SerialComm::write(const std::string& data) {
    std::vector<uint8_t> buffer(data.begin(), data.end());
    return write(buffer);
}

int SerialComm::read(std::vector<uint8_t>& buffer, size_t max_size, unsigned int timeout_ms) {
    if (!is_open()) {
        std::lock_guard<std::mutex> lock(error_mutex_);
        last_error_ = "Port is not open";
        return -1;
    }

    try {
        buffer.resize(max_size);
        boost::system::error_code ec;
        size_t bytes_read = 0;

        if (timeout_ms == 0) {
            // 无超时读取
            bytes_read = serial_port_->read_some(boost::asio::buffer(buffer), ec);
        } else {
            // 带超时读取
            boost::asio::deadline_timer timer(*io_service_);
            timer.expires_from_now(boost::posix_time::milliseconds(timeout_ms));
            
            bool read_complete = false;
            timer.async_wait([&](const boost::system::error_code& error) {
                if (!error && !read_complete) {
                    serial_port_->cancel();
                }
            });

            serial_port_->async_read_some(
                boost::asio::buffer(buffer),
                [&](const boost::system::error_code& error, size_t bytes) {
                    read_complete = true;
                    timer.cancel();
                    ec = error;
                    bytes_read = bytes;
                }
            );

            io_service_->run_one();
            io_service_->reset();
        }

        if (ec) {
            if (ec == boost::asio::error::operation_aborted) {
                std::lock_guard<std::mutex> lock(error_mutex_);
                last_error_ = "Read timeout";
            } else {
                std::lock_guard<std::mutex> lock(error_mutex_);
                last_error_ = "Read error: " + ec.message();
            }
            return -1;
        }

        buffer.resize(bytes_read);
        return static_cast<int>(bytes_read);

    } catch (const std::exception& e) {
        std::lock_guard<std::mutex> lock(error_mutex_);
        last_error_ = "Exception during read: " + std::string(e.what());
        return -1;
    }
}

bool SerialComm::read_line(std::string& line, unsigned int timeout_ms) {
    if (!is_open()) {
        std::lock_guard<std::mutex> lock(error_mutex_);
        last_error_ = "Port is not open";
        return false;
    }

    try {
        boost::system::error_code ec;
        boost::asio::streambuf buffer;

        if (timeout_ms == 0) {
            // 无超时读取
            boost::asio::read_until(*serial_port_, buffer, '\n', ec);
        } else {
            // 带超时读取
            boost::asio::deadline_timer timer(*io_service_);
            timer.expires_from_now(boost::posix_time::milliseconds(timeout_ms));
            
            bool read_complete = false;
            timer.async_wait([&](const boost::system::error_code& error) {
                if (!error && !read_complete) {
                    serial_port_->cancel();
                }
            });

            boost::asio::async_read_until(
                *serial_port_, buffer, '\n',
                [&](const boost::system::error_code& error, size_t bytes) {
                    read_complete = true;
                    timer.cancel();
                    ec = error;
                }
            );

            io_service_->run_one();
            io_service_->reset();
        }

        if (ec) {
            if (ec == boost::asio::error::operation_aborted) {
                std::lock_guard<std::mutex> lock(error_mutex_);
                last_error_ = "Read line timeout";
            } else {
                std::lock_guard<std::mutex> lock(error_mutex_);
                last_error_ = "Read line error: " + ec.message();
            }
            return false;
        }

        std::istream is(&buffer);
        std::getline(is, line);
        return true;

    } catch (const std::exception& e) {
        std::lock_guard<std::mutex> lock(error_mutex_);
        last_error_ = "Exception during read line: " + std::string(e.what());
        return false;
    }
}

void SerialComm::async_write(const std::vector<uint8_t>& data, 
                            std::function<void(const boost::system::error_code&, size_t)> callback) {
    if (!is_open()) {
        if (callback) {
            callback(boost::asio::error::not_connected, 0);
        }
        return;
    }

    boost::asio::async_write(*serial_port_, 
                            boost::asio::buffer(data),
                            [callback](const boost::system::error_code& ec, size_t bytes_transferred) {
                                if (callback) {
                                    callback(ec, bytes_transferred);
                                }
                            });
}

void SerialComm::start_async_read(DataReceivedCallback callback, ErrorCallback error_callback) {
    if (!is_open()) {
        if (error_callback) {
            error_callback("Port is not open");
        }
        return;
    }

    data_callback_ = callback;
    error_callback_ = error_callback;
    is_async_reading_ = true;

    // 开始异步读取
    serial_port_->async_read_some(
        boost::asio::buffer(read_buffer_),
        boost::bind(&SerialComm::async_read_handler, this,
                   boost::asio::placeholders::error,
                   boost::asio::placeholders::bytes_transferred));
}

void SerialComm::stop_async_read() {
    is_async_reading_ = false;
    if (serial_port_ && serial_port_->is_open()) {
        boost::system::error_code ec;
        serial_port_->cancel(ec);
    }
}

void SerialComm::async_read_handler(const boost::system::error_code& error, size_t bytes_transferred) {
    if (!is_async_reading_) {
        return;
    }

    if (!error && bytes_transferred > 0) {
        // 数据接收成功，调用回调函数
        if (data_callback_) {
            std::vector<uint8_t> data(read_buffer_.begin(), read_buffer_.begin() + bytes_transferred);
            data_callback_(data);
        }

        // 继续异步读取
        if (is_async_reading_ && serial_port_ && serial_port_->is_open()) {
            serial_port_->async_read_some(
                boost::asio::buffer(read_buffer_),
                boost::bind(&SerialComm::async_read_handler, this,
                           boost::asio::placeholders::error,
                           boost::asio::placeholders::bytes_transferred));
        }
    } else if (error != boost::asio::error::operation_aborted) {
        // 发生错误（非取消操作）
        if (error_callback_) {
            error_callback_("Async read error: " + error.message());
        }
        is_async_reading_ = false;
    }
}

void SerialComm::flush_input() {
    if (is_open()) {
        boost::system::error_code ec;
        serial_port_->cancel(ec);
    }
}

void SerialComm::flush_output() {
    // boost::asio 串口不直接支持输出缓冲区刷新
    // 这里提供接口以保持兼容性
}

std::vector<std::string> SerialComm::get_available_ports() {
    std::vector<std::string> ports;
    
    // Linux系统下查找串口设备
    const std::vector<std::string> search_paths = {
        "/dev/ttyUSB",  // USB串口
        "/dev/ttyACM",  // ACM串口
        "/dev/ttyS",    // 标准串口
        "/dev/ttyAMA"   // ARM串口
    };

    for (const auto& base_path : search_paths) {
        for (int i = 0; i < 32; ++i) {  // 检查0-31
            std::string port_name = base_path + std::to_string(i);
            struct stat buffer;
            if (stat(port_name.c_str(), &buffer) == 0) {
                ports.push_back(port_name);
            }
        }
    }

    return ports;
}

void SerialComm::set_read_timeout(unsigned int timeout_ms) {
    read_timeout_ms_ = timeout_ms;
}

std::string SerialComm::get_last_error() const {
    std::lock_guard<std::mutex> lock(error_mutex_);
    return last_error_;
}

bool SerialComm::send_protocol_data(uint8_t id, uint8_t data_length, const std::vector<uint8_t>& data) {
    // 检查数据长度是否匹配
    if (data.size() != data_length) {
        std::lock_guard<std::mutex> lock(error_mutex_);
        last_error_ = "Data size mismatch: expected " + std::to_string(data_length) + 
                     ", got " + std::to_string(data.size());
        return false;
    }
    
    // 检查串口是否打开
    if (!is_open()) {
        std::lock_guard<std::mutex> lock(error_mutex_);
        last_error_ = "Serial port is not open";
        return false;
    }

    // 检查数据长度是否超出限制
    if (data_length > ProtocolFrame::MAX_DATA_SIZE) {
        std::lock_guard<std::mutex> lock(error_mutex_);
        last_error_ = "Data length exceeds maximum allowed size: " + std::to_string(data_length);
        return false;
    }

    try {
        // 构建协议帧
        std::vector<uint8_t> frame = build_protocol_frame(id, data);
        
        // 同步发送数据
        int bytes_sent = write(frame);
        
        if (bytes_sent == static_cast<int>(frame.size())) {
            return true;
        } else {
            std::lock_guard<std::mutex> lock(error_mutex_);
            last_error_ = "Failed to send complete frame: sent " + std::to_string(bytes_sent) + 
                         " of " + std::to_string(frame.size()) + " bytes";
            return false;
        }
    } catch (const std::exception& e) {
        std::lock_guard<std::mutex> lock(error_mutex_);
        last_error_ = std::string("Send protocol data error: ") + e.what();
        return false;
    }
}

void SerialComm::start_protocol_receive(ProtocolDataHandler handler, ErrorCallback error_callback) {
    {
        std::lock_guard<std::mutex> lock(protocol_mutex_);
        protocol_handler_ = handler;
        protocol_buffer_.clear();
    }
    
    // 使用协议数据接收回调函数启动异步读取
    start_async_read(
        [this](const std::vector<uint8_t>& data) {
            this->protocol_data_received(data);
        },
        error_callback
    );
}

void SerialComm::stop_protocol_receive() {
    stop_async_read();
    std::lock_guard<std::mutex> lock(protocol_mutex_);
    protocol_handler_ = nullptr;
    protocol_buffer_.clear();
}

std::vector<uint8_t> SerialComm::build_protocol_frame(uint8_t id, const std::vector<uint8_t>& data) {
    std::vector<uint8_t> frame;
    
    // 添加帧头和地址
    frame.push_back(ProtocolFrame::FRAME_HEADER);  // 0xAA
    frame.push_back(ProtocolFrame::ADDRESS);       // 0xFF
    
    // 添加ID和数据长度
    frame.push_back(id);
    frame.push_back(static_cast<uint8_t>(data.size()));
    
    // 添加数据
    frame.insert(frame.end(), data.begin(), data.end());
    
    // 计算校验和
    uint8_t sum_check, add_check;
    calculate_checksum(frame, sum_check, add_check);
    
    // 添加校验字节
    frame.push_back(sum_check);
    frame.push_back(add_check);
    
    return frame;
}

void SerialComm::calculate_checksum(const std::vector<uint8_t>& frame_data, uint8_t& sum_check, uint8_t& add_check) {
    sum_check = 0;
    add_check = 0;
    
    // 从帧头开始到数据区结束进行校验计算
    for (size_t i = 0; i < frame_data.size(); ++i) {
        // 和校验：累加所有字节，只取低8位
        sum_check = static_cast<uint8_t>((sum_check + frame_data[i]) & 0xFF);
        
        // 附加校验：每进行一次加法运算，同时进行一次sum_check的累加
        add_check = static_cast<uint8_t>((add_check + sum_check) & 0xFF);
    }
}

void SerialComm::protocol_data_received(const std::vector<uint8_t>& data) {
    std::lock_guard<std::mutex> lock(protocol_mutex_);
    
    // 将接收到的数据添加到协议缓冲区
    protocol_buffer_.insert(protocol_buffer_.end(), data.begin(), data.end());
    
    // 尝试解析完整的协议帧
    while (parse_protocol_frame(protocol_buffer_)) {
        // parse_protocol_frame会处理找到的帧并从缓冲区中移除
    }
    
    // 如果缓冲区过大，清除一些旧数据以防止内存泄漏
    if (protocol_buffer_.size() > 2048) {
        protocol_buffer_.erase(protocol_buffer_.begin(), protocol_buffer_.begin() + 1024);
    }
}

bool SerialComm::parse_protocol_frame(std::vector<uint8_t>& buffer) {
    // 查找帧头
    auto frame_start = std::find(buffer.begin(), buffer.end(), ProtocolFrame::FRAME_HEADER);
    
    if (frame_start == buffer.end()) {
        // 没找到帧头，清空缓冲区
        buffer.clear();
        return false;
    }
    
    // 移除帧头之前的无效数据
    if (frame_start != buffer.begin()) {
        buffer.erase(buffer.begin(), frame_start);
    }
    
    // 检查是否有足够的数据构成最小帧
    if (buffer.size() < ProtocolFrame::MIN_FRAME_SIZE) {
        return false; // 数据不够，等待更多数据
    }
    
    // 检查地址字节
    if (buffer[1] != ProtocolFrame::ADDRESS) {
        // 地址不匹配，移除当前帧头，继续查找
        buffer.erase(buffer.begin());
        return false;
    }
    
    // 提取帧信息
    uint8_t id = buffer[2];
    uint8_t data_length = buffer[3];
    
    // 计算完整帧的长度
    size_t frame_length = 4 + data_length + 2; // 帧头+地址+ID+长度+数据+校验(2字节)
    
    if (buffer.size() < frame_length) {
        return false; // 数据不够，等待更多数据
    }
    
    // 提取数据部分（用于校验计算的帧数据）
    std::vector<uint8_t> frame_for_check(buffer.begin(), buffer.begin() + 4 + data_length);
    
    // 提取实际数据
    std::vector<uint8_t> actual_data(buffer.begin() + 4, buffer.begin() + 4 + data_length);
    
    // 提取接收到的校验值
    uint8_t received_sum_check = buffer[4 + data_length];
    uint8_t received_add_check = buffer[4 + data_length + 1];
    
    // 计算校验值
    uint8_t calculated_sum_check, calculated_add_check;
    calculate_checksum(frame_for_check, calculated_sum_check, calculated_add_check);
    
    // 验证校验值
    if (received_sum_check == calculated_sum_check && received_add_check == calculated_add_check) {
        // 校验通过，处理数据
        if (protocol_handler_) {
            protocol_handler_(id, actual_data);
        }
        
        // 从缓冲区中移除已处理的帧
        buffer.erase(buffer.begin(), buffer.begin() + frame_length);
        return true;
    } else {
        // 校验失败，移除当前帧头，继续查找下一帧
        buffer.erase(buffer.begin());
        return false;
    }
}

void SerialComm::start_io_service() {
    if (!is_running_) {
        is_running_ = true;
        io_thread_ = std::make_unique<std::thread>([this]() {
            while (is_running_) {
                try {
                    io_service_->run();
                    io_service_->reset();
                    if (is_running_) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    }
                } catch (const std::exception& e) {
                    if (error_callback_) {
                        error_callback_("IO service error: " + std::string(e.what()));
                    }
                    break;
                }
            }
        });
    }
}

void SerialComm::stop_io_service() {
    if (is_running_) {
        is_running_ = false;
        if (io_service_) {
            io_service_->stop();
        }
        if (io_thread_ && io_thread_->joinable()) {
            io_thread_->join();
        }
        io_thread_.reset();
    }
}

} // namespace serial_comm