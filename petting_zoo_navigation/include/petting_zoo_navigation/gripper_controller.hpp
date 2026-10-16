#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "control_msgs/action/gripper_command.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "vision_msgs/msg/bounding_box2_d_array.hpp"
#include "mirte_msgs/msg/intensity_digital.hpp"


// Opens the gripper while enabled. Closes it on its own while the disliked apple is in view,
// or when the button is pressed shortly after the liked apple was seen. After a button close it
// waits, asks the arm to move aside, and opens again to drop the apple.
class gripperController : public rclcpp::Node{
    public:
        gripperController();
    private:
        using GripperCommand = control_msgs::action::GripperCommand;

        void enable_callback(const std_msgs::msg::Bool::SharedPtr msg);
        void disliked_callback(const vision_msgs::msg::BoundingBox2DArray::SharedPtr msg);
        void check_disliked_lost();
        void send(double position);
        void liked_callback(const vision_msgs::msg::BoundingBox2DArray::SharedPtr msg);
        void button_callback(const mirte_msgs::msg::IntensityDigital::SharedPtr msg);
        void update_drop();
        void set_dropping(bool on);

        rclcpp_action::Client<GripperCommand>::SharedPtr gripper_client_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_subscriber_;
        rclcpp::Subscription<vision_msgs::msg::BoundingBox2DArray>::SharedPtr disliked_subscriber_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<vision_msgs::msg::BoundingBox2DArray>::SharedPtr liked_subscriber_;
        rclcpp::Subscription<mirte_msgs::msg::IntensityDigital>::SharedPtr button_subscriber_;
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr aside_publisher_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr dropping_publisher_;   //tells the FSM to hold the apple state

        bool enabled_ = false;
        bool closed_ = false;              //closed because the disliked apple is in view
        int disliked_streak_ = 0;             //disliked apple detections in a row
        rclcpp::Time disliked_last_seen_;

        double open_position_ = -0.3;
        double closed_position_ = 0.1;
        int disliked_frames_ = 3;             //this many in a row before closing
        double disliked_lost_timeout_ = 1.0;  //s without the disliked apple before opening again

        bool button_closed_ = false;       //closed by the button, stays closed until the next enable
        rclcpp::Time liked_last_seen_{0, 0, RCL_ROS_TIME};
        bool liked_seen_ = false;            //no liked apple seen yet since start
        double liked_memory_ = 20.0;         //s, the button works this long after the last liked apple
        bool button_pressed_value_ = false;

        int drop_step_ = 0;                //0 = idle, 1 = waiting to move the arm aside, 2 = waiting to open
        rclcpp::Time drop_step_time_;
        double drop_delay_ = 2.0;          //s between the button closing the gripper and the arm moving aside
        double aside_time_ = 2.0;          //s for the arm to reach the side before the gripper opens
};
