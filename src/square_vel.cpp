#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <chrono>
#include <iostream>
#include <sstream>
#include <cmath>

using namespace std::chrono;
using namespace std::chrono_literals;

template <typename T>
T clamp(T val, T min, T max){
    return std::max(min, std::min(val, max));
}

class SquareVel: public rclcpp::Node
{
public:
	SquareVel() : Node("square_vel")
	{
        RCLCPP_INFO(this->get_logger(), "Starting square_vel demo");

        // x
        this->declare_parameter<float>("length", 2.0);
        this->get_parameter("length", length);

        // y
        this->declare_parameter<float>("width", 2.0);
        this->get_parameter("width", width);

        // z
        this->declare_parameter<float>("alt", -3.0);
        this->get_parameter("alt", alt);

        this->declare_parameter<float>("x_offset", 2.0);
        this->get_parameter("x_offset", x_offset);

        this->declare_parameter<float>("y_offset", 2.0);
        this->get_parameter("y_offset", y_offset);

        RCLCPP_INFO(this->get_logger(), "Dimensions");
        RCLCPP_INFO(this->get_logger(), "Length: %f", length);
        RCLCPP_INFO(this->get_logger(), "Width: %f", width);
        RCLCPP_INFO(this->get_logger(), "Altitude: %f", alt);
        RCLCPP_INFO(this->get_logger(), "X Offset: %f", x_offset);
        RCLCPP_INFO(this->get_logger(), "Y Offset: %f", y_offset);

        // QoS
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

        // Square mission waypoints
        way_pt_idx = 0;
        waypts << x_offset, y_offset, alt, 0.0,
                  x_offset + length, y_offset, alt, 0.0,
                  x_offset + length, y_offset + width, alt, 0.0,
                  x_offset , y_offset + width, alt, 0.0;

        // Pubs and Subs
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", qos);
        pos_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("pose", qos, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg){
            pos_msg_ = msg->pose.position;
        });

		timer_ = this->create_wall_timer(100ms, std::bind(&SquareVel::timer_callback, this));
	}


private:
	std::atomic<uint64_t> timestamp_;
	rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pos_sub_;
    geometry_msgs::msg::Point pos_msg_;

    float length;
    float width;
    float alt;
    float x_offset;
    float y_offset;

    uint8_t way_pt_idx;
	const float POS_TOL_ = 0.5; 
    Eigen::Matrix<float, 4, 4> waypts;
	bool waypt_reached = false;

    void timer_callback();
	bool has_reached_pose(const Eigen::Vector4f& target_pos);
    void publish_cmd_vel(const Eigen::Vector4f& target_pos);
};

void SquareVel::timer_callback()
{
    // compute the new velocity based on where the drone is wrt to the next waypoint
    publish_cmd_vel(waypts.row(way_pt_idx));

    // error calculation
    waypt_reached = this->has_reached_pose(waypts.row(way_pt_idx));
    if (waypt_reached) {
        way_pt_idx = (way_pt_idx + 1) % 4;
        RCLCPP_INFO(this->get_logger(), "Waypoint %d reached", way_pt_idx);
    }
}

void SquareVel::publish_cmd_vel(const Eigen::Vector4f& target_pos){
	const float kP = 1.0;

	const float err_x = (pos_msg_.x - target_pos[0]);
	const float err_y = (pos_msg_.y - target_pos[1]);
	const float err_z = (pos_msg_.z - target_pos[2]);

    const float vx = clamp(-kP * err_x, -2.0f, 2.0f);
    const float vy = clamp(-kP * err_y, -2.0f, 2.0f);
    const float vz = clamp(-kP * err_z, -2.0f, 2.0f);

    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.stamp = this->now();
    cmd_vel.twist.linear.x = vx;
    cmd_vel.twist.linear.y = vy;
    cmd_vel.twist.linear.z = vz;
    cmd_vel.twist.angular.x = 0.0;
    cmd_vel.twist.angular.y = 0.0;
    cmd_vel.twist.angular.z = 0.0;
    cmd_vel_pub_->publish(cmd_vel);
}

bool SquareVel::has_reached_pose(const Eigen::Vector4f& target_pos)
{
	const float err_x = std::abs(pos_msg_.x - target_pos[0]);
	const float err_y = std::abs(pos_msg_.y - target_pos[1]);
	const float err_z = std::abs(pos_msg_.z - target_pos[2]);

	return err_x < POS_TOL_ && err_y < POS_TOL_ && err_z < POS_TOL_;
}

int main(int argc, char *argv[])
{
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SquareVel>());
	rclcpp::shutdown();
	return 0;
}
