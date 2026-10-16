#pragma once

#include <random>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"



class randomVelocityPublisher : public rclcpp::Node{
    public:

        randomVelocityPublisher();

    private:

        void pick_new_velocity();
        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
        void publish_velocity();
        void refresh_params();

        void enable_callback(const std_msgs::msg::Bool::SharedPtr msg);

        //Random walk
        std::mt19937 rng_{std::random_device{}()};
        std::uniform_real_distribution<double> lin_dist_{-0.3,0.3};
        std::uniform_real_distribution<double> ang_dist_{-0.3,0.3};
    

        //Ros Members
        rclcpp::TimerBase::SharedPtr pick_timer_;
        rclcpp::TimerBase::SharedPtr publish_timer_;
        rclcpp::TimerBase::SharedPtr straighten_timer_;   // add this

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr random_velocity_publisher_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_subscriber_; 

        geometry_msgs::msg::Twist current_cmd_;

        double front_dist_ = std::numeric_limits<double>::infinity();
        double left_clear_  = std::numeric_limits<double>::infinity();  // gap between robot edge and nearest obstacle (m)
        double right_clear_ = std::numeric_limits<double>::infinity();
        double front_offset_ = 0.0;   // scan angle (rad) that points to robot-front; set from param
        bool flip_scan_ = false;      // true if the lidar's angles run clockwise (mirrored scan)
        double min_range_ = 0.15;     // ignore lidar returns closer than this (m); set from param
        double turn_dir_ = 1;
        bool avoiding_ = false;           // true while committed to an in-place turn

        double robot_half_width_ = 0.19;  // half robot width + margin (m)
        double side_margin_ = 0.10;       // min free space beside the robot while steering (m)
        double arm_reach_ = 0.35;         // how far the lowered arm sticks out ahead of the lidar (m)
        double stop_margin_ = 0.15;       // free space to keep between arm tip and obstacle (m)
        double slow_margin_ = 0.30;       // start slowing down this far before the stop point (m)

        bool enabled_ = false;
};