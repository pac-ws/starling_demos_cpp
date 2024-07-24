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

// Square mission parameters
#define S_Z -1.0
#define S_X 2.0

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
        // Parameters
        this->declare_parameter<std::string>("namespace", "r9");
        std::string ns;
        this->get_parameter("namespace", ns);
        
        // QoS
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
        
        // Fixed altitude mission
        waypts << 1.0, 1.0, S_Z, 0.0,

        // Pubs and Subs
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(ns + "/cmd_vel", qos);
        pos_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(ns + "/pose", qos, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg){
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
	std::cout << "Starting square vel demo node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<SquareVel>());
	rclcpp::shutdown();
	return 0;
}
