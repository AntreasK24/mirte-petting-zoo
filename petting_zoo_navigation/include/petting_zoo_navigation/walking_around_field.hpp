#pragma once

#include <random>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"


// Random walk that avoids obstacles all around the robot, not just in front.
// Every lidar point within influence_radius pushes the robot away from it
// (a repulsive field); the robot steers towards "forward + push". A hard
// front stop stays as a safety backup.
class fieldWalker : public rclcpp::Node{
    public:

        fieldWalker();

    private:

        void pick_new_velocity();
        void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
        void publish_velocity();
        void refresh_params();

        void enable_callback(const std_msgs::msg::Bool::SharedPtr msg);

        //Random walk
        std::mt19937 rng_{std::random_device{}()};
        std::uniform_real_distribution<double> ang_dist_{-0.3,0.3};

        //Ros Members
        rclcpp::TimerBase::SharedPtr pick_timer_;
        rclcpp::TimerBase::SharedPtr publish_timer_;
        rclcpp::TimerBase::SharedPtr straighten_timer_;

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_subscriber_;

        geometry_msgs::msg::Twist current_cmd_;

        //Scan results
        double front_dist_ = std::numeric_limits<double>::infinity();
        double left_clear_  = std::numeric_limits<double>::infinity();  // gap between robot edge and nearest obstacle (m)
        double right_clear_ = std::numeric_limits<double>::infinity();
        double rep_x_ = 0.0;              // repulsion vector: points away from nearby obstacles
        double rep_y_ = 0.0;

        //Lidar mounting
        double front_offset_ = 0.0;       // scan angle (rad) that points to robot-front
        bool flip_scan_ = false;          // true if the lidar's angles run clockwise (mirrored scan)
        double min_range_ = 0.15;         // ignore lidar returns closer than this (m)

        //Robot shape
        double robot_half_width_ = 0.19;  // half robot width + margin (m)
        double arm_reach_ = 0.35;         // how far the lowered arm sticks out ahead of the lidar (m)

        //Front safety stop
        double stop_margin_ = 0.15;       // free space to keep between arm tip and obstacle (m)
        double slow_margin_ = 0.30;       // start slowing down this far before the stop point (m)
        double turn_speed_ = 1.0;         // rad/s when turning in place away from a blocked front
        double turn_dir_ = 1;
        bool avoiding_ = false;           // true while committed to an in-place turn

        //Repulsive field
        double influence_radius_ = 0.6;   // obstacles closer than this push the robot away (m)
        double rep_gain_ = 1.5;           // how strongly obstacles push compared to driving forward
        double max_ang_ = 1.0;            // max steering rate from the field (rad/s)

        bool enabled_ = false;
};
