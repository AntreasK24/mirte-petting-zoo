#include <chrono>
#include <functional>
#include <memory>

#include "petting_zoo_navigation/gripper_controller.hpp"

using namespace std::chrono_literals;

gripperController::gripperController()
: Node("gripper_controller")
{
    open_position_ = this->declare_parameter<double>("open_position", -0.3);
    closed_position_ = this->declare_parameter<double>("closed_position", 0.1);
    disliked_frames_ = this->declare_parameter<int>("disliked_frames", 3);
    disliked_lost_timeout_ = this->declare_parameter<double>("disliked_lost_timeout", 1.0);
    button_pressed_value_ = this->declare_parameter<bool>("button_pressed_value", false);
    liked_memory_ = this->declare_parameter<double>("liked_memory", liked_memory_);
    drop_delay_ = this->declare_parameter<double>("drop_delay", drop_delay_);
    aside_time_ = this->declare_parameter<double>("aside_time", aside_time_);

    disliked_last_seen_ = this->now();
    drop_step_time_ = this->now();

    gripper_client_ = rclcpp_action::create_client<GripperCommand>(
        this, "/mirte_master_gripper_controller/gripper_cmd");

    enable_subscriber_ = this->create_subscription<std_msgs::msg::Bool>(
        "gripper_controller/enable", rclcpp::QoS(1).transient_local(),
        std::bind(&gripperController::enable_callback, this, std::placeholders::_1));

    disliked_subscriber_ = this->create_subscription<vision_msgs::msg::BoundingBox2DArray>(
        "detections/apples/green", 10,
        std::bind(&gripperController::disliked_callback, this, std::placeholders::_1));

    liked_subscriber_ = this->create_subscription<vision_msgs::msg::BoundingBox2DArray>(
        "detections/apples/red", 10,
        std::bind(&gripperController::liked_callback, this, std::placeholders::_1));

    button_subscriber_ = this->create_subscription<mirte_msgs::msg::IntensityDigital>(
        "button", 10,
        std::bind(&gripperController::button_callback, this, std::placeholders::_1));

    aside_publisher_ = this->create_publisher<std_msgs::msg::Empty>("arm_track_controller/move_aside", 10);
    dropping_publisher_ = this->create_publisher<std_msgs::msg::Bool>(
        "gripper_controller/dropping", rclcpp::QoS(1).transient_local());
    set_dropping(false);

    timer_ = this->create_wall_timer(100ms, [this]{ check_disliked_lost(); update_drop(); });
}

void gripperController::enable_callback(const std_msgs::msg::Bool::SharedPtr msg){
    button_closed_ = false;
    if(drop_step_ != 0) set_dropping(false);
    drop_step_ = 0;
    enabled_ = msg->data;
    disliked_streak_ = 0;
    closed_ = false;
    //open while enabled, closed otherwise
    send(enabled_ ? open_position_ : closed_position_);
}

void gripperController::disliked_callback(const vision_msgs::msg::BoundingBox2DArray::SharedPtr msg){
    if(button_closed_) return;
    if(!enabled_ || msg->boxes.empty()) return;

    //detector runs at ~5 Hz, so a gap over 0.5 s means the disliked apple was lost in between
    if((this->now() - disliked_last_seen_).seconds() < 0.5) ++disliked_streak_;
    else disliked_streak_ = 1;
    disliked_last_seen_ = this->now();

    if(!closed_ && disliked_streak_ >= disliked_frames_){
        RCLCPP_INFO(this->get_logger(), "Disliked apple seen %d frames in a row, closing gripper", disliked_streak_);
        closed_ = true;
        send(closed_position_);
    }
}

void gripperController::check_disliked_lost(){
    if(!enabled_ || !closed_) return;
    double since_disliked = (this->now() - disliked_last_seen_).seconds();
    if(since_disliked > disliked_lost_timeout_){
        RCLCPP_INFO(this->get_logger(), "No disliked apple for %.1f s, opening gripper", since_disliked);
        closed_ = false;
        send(open_position_);
    }
}

void gripperController::send(double position){
    if(!gripper_client_->action_server_is_ready()){
        RCLCPP_WARN(this->get_logger(), "Gripper action server not available");
        return;
    }
    GripperCommand::Goal goal;
    goal.command.position = position;
    goal.command.max_effort = 1.0;
    gripper_client_->async_send_goal(goal);   //a new goal replaces the previous one
}


void gripperController::liked_callback(const vision_msgs::msg::BoundingBox2DArray::SharedPtr msg){
    if(msg->boxes.empty()) return;
    liked_last_seen_ = this->now();
    liked_seen_ = true;
}

void gripperController::button_callback(const mirte_msgs::msg::IntensityDigital::SharedPtr msg){
    if(msg->value != button_pressed_value_ || button_closed_) return;

    double since_liked = (this->now() - liked_last_seen_).seconds();
    if(!liked_seen_ || since_liked > liked_memory_){
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Button pressed, but no liked apple for %.0f s", since_liked);
        return;
    }
    RCLCPP_INFO(this->get_logger(), "Button pressed, liked apple seen %.1f s ago, closing gripper", since_liked);
    button_closed_ = true;
    send(closed_position_);
    drop_step_ = 1;
    drop_step_time_ = this->now();
    set_dropping(true);
}

//after a button close: wait drop_delay, move the arm aside, wait aside_time, open to drop the apple
void gripperController::update_drop(){
    if(drop_step_ == 0 || !enabled_) return;
    double since = (this->now() - drop_step_time_).seconds();
    if(drop_step_ == 1 && since > drop_delay_){
        RCLCPP_INFO(this->get_logger(), "Moving the arm aside");
        aside_publisher_->publish(std_msgs::msg::Empty());
        drop_step_ = 2;
        drop_step_time_ = this->now();
    } else if(drop_step_ == 2 && since > aside_time_){
        RCLCPP_INFO(this->get_logger(), "Arm is aside, opening gripper");
        send(open_position_);
        drop_step_ = 0;
        set_dropping(false);
    }
}

void gripperController::set_dropping(bool on){
    std_msgs::msg::Bool m;
    m.data = on;
    dropping_publisher_->publish(m);
}

int main(int argc, char * argv[]){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<gripperController>());
    rclcpp::shutdown();
    return 0;
}
