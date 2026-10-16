#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "vision_msgs/msg/bounding_box2_d.hpp"
#include "vision_msgs/msg/bounding_box2_d_array.hpp"
#include "petting_zoo_fsm/state.hpp"


class FSMNode : public rclcpp::Node {
    public:
        FSMNode();
        void switchState(const std::string& name);

        bool human_visible_ = false;
        rclcpp::Time last_seen_;
        vision_msgs::msg::BoundingBox2D last_box_;

        bool apple_in_hand_ = false;
        bool dropping_ = false;          //gripper is moving the liked apple aside and dropping it, stay in the apple state
        rclcpp::Time apple_last_seen_;
        double apple_lost_timeout_ = 3.0;

        int liked_streak_ = 0;             //liked apple-in-hand detections in a row
        int disliked_streak_ = 0;           //disliked apple-in-hand detections in a row
        int apple_track_frames_ = 3;     //this many in a row before the arm reacts to the apple
        rclcpp::Time liked_last_seen_;
        rclcpp::Time disliked_last_seen_;

        rclcpp::Time state_entered_;
        rclcpp::Time ignore_humans_until_;

        double lost_timeout_ = 10.0;
        double max_track_time_ = 30.0;
        double cooldown_ = 10.0;

        void setWalking(bool on) {std_msgs::msg::Bool m; m.data = on; walk_enable_pub_->publish(m);}
        void setTracking(bool on) {std_msgs::msg::Bool m; m.data = on; arm_enable_pub_->publish(m);}
        void setGripper(bool on) {std_msgs::msg::Bool m; m.data = on; gripper_enable_pub_->publish(m);}
        //what the arm follows: "" = human, "liked" = the liked apple, "hold" = stay still
        void setAppleTarget(const std::string& colour) {std_msgs::msg::String m; m.data = colour; arm_apple_pub_->publish(m);}

    private:
        void tick();

        template <typename T>
        void addState(const std::string& name){
            states_[name] = std::make_unique<T>(this);
        }

        void appleCallback(const vision_msgs::msg::BoundingBox2DArray::SharedPtr msg, const std::string& colour);


        std::unordered_map<std::string, std::unique_ptr<State>> states_;
        State* current_ = nullptr;
        std::string current_name_;
        std::string pending_;
        bool insideHuman(const vision_msgs::msg::BoundingBox2D& apple) const;

        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<vision_msgs::msg::BoundingBox2DArray>::SharedPtr box_sub_;

        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr walk_enable_pub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr arm_enable_pub_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr arm_apple_pub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_enable_pub_;

        rclcpp::Subscription<vision_msgs::msg::BoundingBox2DArray>::SharedPtr liked_apple_sub_;
        rclcpp::Subscription<vision_msgs::msg::BoundingBox2DArray>::SharedPtr disliked_apple_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr dropping_sub_;



};