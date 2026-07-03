#include "pid_control_pkg/pid_control.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/exceptions.h>
#include <cmath>
#include <algorithm>

namespace pid_control_pkg {

PidControl::PidControl(const rclcpp::NodeOptions& options)
    : rclcpp::Node("pid_control_node", options)
{
    declare_parameter("source_frame", source_frame_);
    declare_parameter("target_frame", target_frame_);
    declare_parameter("kp_x", kp_x_);
    declare_parameter("ki_x", ki_x_);
    declare_parameter("kd_x", kd_x_);
    declare_parameter("kp_y", kp_y_);
    declare_parameter("ki_y", ki_y_);
    declare_parameter("kd_y", kd_y_);
    declare_parameter("kp_z", kp_z_);
    declare_parameter("ki_z", ki_z_);
    declare_parameter("kd_z", kd_z_);
    declare_parameter("kp_yaw", kp_yaw_);
    declare_parameter("ki_yaw", ki_yaw_);
    declare_parameter("kd_yaw", kd_yaw_);
    declare_parameter("v_max_xy", v_max_xy_);
    declare_parameter("v_max_z", v_max_z_);
    declare_parameter("v_max_yaw", v_max_yaw_);
    declare_parameter("i_max_xy", i_max_xy_);
    declare_parameter("i_max_z", i_max_z_);
    declare_parameter("i_max_yaw", i_max_yaw_);
    declare_parameter("pos_deadzone", pos_deadzone_);
    declare_parameter("yaw_deadzone", yaw_deadzone_);

    source_frame_ = get_parameter("source_frame").as_string();
    target_frame_ = get_parameter("target_frame").as_string();
    kp_x_ = get_parameter("kp_x").as_double();
    ki_x_ = get_parameter("ki_x").as_double();
    kd_x_ = get_parameter("kd_x").as_double();
    kp_y_ = get_parameter("kp_y").as_double();
    ki_y_ = get_parameter("ki_y").as_double();
    kd_y_ = get_parameter("kd_y").as_double();
    kp_z_ = get_parameter("kp_z").as_double();
    ki_z_ = get_parameter("ki_z").as_double();
    kd_z_ = get_parameter("kd_z").as_double();
    kp_yaw_ = get_parameter("kp_yaw").as_double();
    ki_yaw_ = get_parameter("ki_yaw").as_double();
    kd_yaw_ = get_parameter("kd_yaw").as_double();
    v_max_xy_ = get_parameter("v_max_xy").as_double();
    v_max_z_ = get_parameter("v_max_z").as_double();
    v_max_yaw_ = get_parameter("v_max_yaw").as_double();
    i_max_xy_ = get_parameter("i_max_xy").as_double();
    i_max_z_ = get_parameter("i_max_z").as_double();
    i_max_yaw_ = get_parameter("i_max_yaw").as_double();
    pos_deadzone_ = get_parameter("pos_deadzone").as_double();
    yaw_deadzone_ = get_parameter("yaw_deadzone").as_double();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    height_sub_ = create_subscription<std_msgs::msg::Int16>(
        "/height", 5,
        [this](const std_msgs::msg::Int16::SharedPtr msg) { heightCallback(msg); });

    target_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
        "/target_position", 5,
        [this](const std_msgs::msg::Float32MultiArray::SharedPtr msg) { targetCallback(msg); });

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/velocity_map", 5);

    last_time_ = now();

    timer_ = create_wall_timer(
        std::chrono::milliseconds(50),  // 20 Hz
        [this]() { timerCallback(); });

    RCLCPP_INFO(get_logger(), "PidControl initialized: source_frame=%s target_frame=%s",
        source_frame_.c_str(), target_frame_.c_str());
}

void PidControl::timerCallback()
{
    updateStateFromTF();
    computeAndPublish();
}

void PidControl::heightCallback(const std_msgs::msg::Int16::SharedPtr msg)
{
    z_ = static_cast<double>(msg->data) / 100.0;
}

void PidControl::targetCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
    if (msg->data.size() < 4) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "target_position requires 4 values [x_cm, y_cm, z_cm, yaw_deg]");
        return;
    }
    tx_ = msg->data[0] / 100.0;
    ty_ = msg->data[1] / 100.0;
    tz_ = msg->data[2] / 100.0;
    tyaw_ = msg->data[3] * M_PI / 180.0;
}

void PidControl::updateStateFromTF()
{
    try {
        geometry_msgs::msg::TransformStamped t =
            tf_buffer_->lookupTransform(source_frame_, target_frame_, tf2::TimePointZero);
        x_ = t.transform.translation.x;
        y_ = t.transform.translation.y;

        tf2::Quaternion q;
        tf2::fromMsg(t.transform.rotation, q);
        double roll, pitch;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw_);
    } catch (const tf2::TransformException& ex) {
        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 5000,
            "TF lookup failed: %s", ex.what());
    }
}

static inline double clampd(double v, double lo, double hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

void PidControl::computeAndPublish()
{
    rclcpp::Time now_time = now();
    double dt = (now_time - last_time_).seconds();
    if (dt <= 0.0 || dt > 0.5) dt = 1e-3;

    double ex = tx_ - x_;
    double ey = ty_ - y_;
    double ez = tz_ - z_;
    double eyaw = tyaw_ - yaw_;

    while (eyaw > M_PI) eyaw -= 2 * M_PI;
    while (eyaw < -M_PI) eyaw += 2 * M_PI;

    if (std::fabs(ex) < pos_deadzone_) ex = 0.0;
    if (std::fabs(ey) < pos_deadzone_) ey = 0.0;
    if (std::fabs(ez) < pos_deadzone_) ez = 0.0;
    if (std::fabs(eyaw) < yaw_deadzone_) eyaw = 0.0;

    double dx = (ex - prev_ex_) / dt;
    double dy = (ey - prev_ey_) / dt;
    double dz = (ez - prev_ez_) / dt;
    double dyaw = (eyaw - prev_eyaw_) / dt;

    double ux_pd = kp_x_ * ex + kd_x_ * dx;
    double uy_pd = kp_y_ * ey + kd_y_ * dy;
    double uz_pd = kp_z_ * ez + kd_z_ * dz;
    double uyaw_pd = kp_yaw_ * eyaw + kd_yaw_ * dyaw;

    auto cond_integrate = [](double& integ, double err, double dt_,
                              double pd, double vmax, double imax) {
        bool saturated_same_sign = (pd > vmax && err > 0.0) || (pd < -vmax && err < 0.0);
        if (!saturated_same_sign) integ += err * dt_;
        integ = std::max(-imax, std::min(imax, integ));
    };
    cond_integrate(ix_, ex, dt, ux_pd, v_max_xy_, i_max_xy_);
    cond_integrate(iy_, ey, dt, uy_pd, v_max_xy_, i_max_xy_);
    cond_integrate(iz_, ez, dt, uz_pd, v_max_z_, i_max_z_);
    cond_integrate(iyaw_int_, eyaw, dt, uyaw_pd, v_max_yaw_, i_max_yaw_);

    double ux = clampd(ux_pd + ki_x_ * ix_, -v_max_xy_, v_max_xy_);
    double uy = clampd(uy_pd + ki_y_ * iy_, -v_max_xy_, v_max_xy_);
    double uz = clampd(uz_pd + ki_z_ * iz_, -v_max_z_, v_max_z_);
    double uyaw = clampd(uyaw_pd + ki_yaw_ * iyaw_int_, -v_max_yaw_, v_max_yaw_);

    prev_ex_ = ex;
    prev_ey_ = ey;
    prev_ez_ = ez;
    prev_eyaw_ = eyaw;
    last_time_ = now_time;

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = ux;
    cmd.linear.y = uy;
    cmd.linear.z = uz;
    cmd.angular.z = uyaw;
    cmd_pub_->publish(cmd);
}

} // namespace pid_control_pkg

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<pid_control_pkg::PidControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
