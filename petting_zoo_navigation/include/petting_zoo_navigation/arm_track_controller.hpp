#pragma once

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "vision_msgs/msg/bounding_box2_d.hpp"
#include "vision_msgs/msg/bounding_box2_d_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"





class armTrackerController : public rclcpp::Node{
    public:
        armTrackerController();
    private:

        void publish_joint_states();
        void arm_target_callback(const trajectory_msgs::msg::JointTrajectoryPoint::SharedPtr msg);
        void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
        void human_bounding_box_callback(const vision_msgs::msg::BoundingBox2DArray::SharedPtr msg);
        void enable_callback(const std_msgs::msg::Bool::SharedPtr msg);
        void apple_callback(const vision_msgs::msg::BoundingBox2DArray::SharedPtr msg, const std::string& colour);
        void apple_colour_callback(const std_msgs::msg::String::SharedPtr msg);
        void track_biggest(const vision_msgs::msg::BoundingBox2DArray& msg);
        void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
        void compensate_base_rotation(double dt);
        void move_aside_callback(const std_msgs::msg::Empty::SharedPtr msg);


        rclcpp::TimerBase::SharedPtr publish_timer_;
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_trajectory_publisher_;
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
        rclcpp::Subscription<vision_msgs::msg::BoundingBox2DArray>::SharedPtr human_bounding_box_subscriber_;
        rclcpp::Subscription<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr arm_target_subscriber_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_subscriber_;
        rclcpp::Subscription<vision_msgs::msg::BoundingBox2DArray>::SharedPtr liked_apple_subscriber_;
        rclcpp::Subscription<vision_msgs::msg::BoundingBox2DArray>::SharedPtr disliked_apple_subscriber_;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr apple_colour_subscriber_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
        rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr move_aside_subscriber_;


        std::vector<std::string> joint_names_{"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_joint"};
        std::vector<double> start_pose_{0.0, 0.0, -1.3, 0.0};
        std::vector<double> target_ = start_pose_;
        std::vector<double> current_ = start_pose_;
        bool have_joint_states_ = false;
        bool at_start_ = false;

        int sends_left_ = 3;
        double move_time_ = 0.2;

        double image_width_ = 640.0;
        double image_height_ = 480.0;
        double rad_per_pixel_ = 1.047/640.0;
        double gain_ = 0.7;
        double deadband_px_ = 15.0;
        bool reached(const std::vector<double>& pose, double tol = 0.05) const;
        bool enabled_ = false;

        std::string apple_colour_;       // "" = follow the human, "liked"/"disliked" = follow that apple

        bool aside_ = false;             // moved aside to drop the apple, tracking paused until a new target
        double aside_pan_ = 0.8;         // rad added to shoulder_pan_joint when moving aside

        bool counter_rotate_ = true;          // turn the arm against base rotation
        double base_yaw_rate_ = 0.0;
        rclcpp::Time base_cmd_time_{0, 0, RCL_ROS_TIME};
        double cmd_timeout_ = 0.5;            // s, treat the base as stopped after this long without a command
        double counter_rotate_ratio_ = 1.0;   // fraction of base rotation the arm undoes
        double min_yaw_rate_ = 0.05;          // rad/s, ignore tiny turn commands below this
        double max_counter_pan_ = 1.0;        // rad, how far the arm may lean while searching



};
