#include "petting_zoo_fsm/fsm_node.hpp"
#include "petting_zoo_fsm/states/state_searching.hpp"
#include "petting_zoo_fsm/states/state_tracking.hpp"
#include "petting_zoo_fsm/states/state_apple.hpp"


FSMNode::FSMNode() : Node("fsm_node"){
    last_seen_ = now();
    ignore_humans_until_ = now();
    state_entered_ = now();

    auto latched = rclcpp::QoS(1).transient_local();
    walk_enable_pub_ = create_publisher<std_msgs::msg::Bool>("walking_around/enable", latched);
    arm_enable_pub_  = create_publisher<std_msgs::msg::Bool>("arm_track_controller/enable", latched);
    arm_apple_pub_   = create_publisher<std_msgs::msg::String>("arm_track_controller/apple_colour", latched);
    gripper_enable_pub_ = create_publisher<std_msgs::msg::Bool>("gripper_controller/enable", latched);


    apple_last_seen_ = now();
    liked_last_seen_ = now();
    disliked_last_seen_ = now();


    //States
    addState<SearchingState>("searching");
    addState<TrackingState>("tracking");
    addState<AppleState>("apple");


    liked_apple_sub_   = create_subscription<vision_msgs::msg::BoundingBox2DArray>("detections/apples/red", 10,
        [this](vision_msgs::msg::BoundingBox2DArray::SharedPtr msg){ appleCallback(msg, "liked"); });
    disliked_apple_sub_ = create_subscription<vision_msgs::msg::BoundingBox2DArray>("detections/apples/green", 10,
        [this](vision_msgs::msg::BoundingBox2DArray::SharedPtr msg){ appleCallback(msg, "disliked"); });

    dropping_sub_ = create_subscription<std_msgs::msg::Bool>("gripper_controller/dropping", rclcpp::QoS(1).transient_local(),
        [this](std_msgs::msg::Bool::SharedPtr msg){
            if(msg->data != dropping_) RCLCPP_INFO(get_logger(), "%s", msg->data ? "Dropping the apple, holding the apple state" : "Apple dropped");
            dropping_ = msg->data;
        });


    box_sub_ = create_subscription<vision_msgs::msg::BoundingBox2DArray>(
        "detections/humans",10,
        [this](vision_msgs::msg::BoundingBox2DArray::SharedPtr msg){
            if(msg->boxes.empty()) return;
            last_box_ = msg->boxes[0];
            for(const auto& b : msg->boxes)
                if(b.size_x * b.size_y > last_box_.size_x * last_box_.size_y) last_box_ = b;
            human_visible_ = true;
            last_seen_ = now();
        }
    );

    switchState("searching");
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] { tick(); });
}

void FSMNode::appleCallback(const vision_msgs::msg::BoundingBox2DArray::SharedPtr msg, const std::string& colour){
    if((now() - last_seen_).seconds() > 0.5) return;
    for(const auto& apple : msg->boxes){
        if(insideHuman(apple)){
            //detector runs at ~5 Hz, so a gap over 0.5 s means the apple was lost in between
            bool liked = (colour == "liked");
            int& streak = liked ? liked_streak_ : disliked_streak_;
            rclcpp::Time& last = liked ? liked_last_seen_ : disliked_last_seen_;
            if((now() - last).seconds() < 0.5) ++streak;
            else streak = 1;
            last = now();
            apple_in_hand_ = true;
            apple_last_seen_ = now();
            return;
        }
    }
}


bool FSMNode::insideHuman(const vision_msgs::msg::BoundingBox2D& apple) const{
    const auto& h = last_box_;
    double half_w = h.size_x / 2.0 * 1.3;   //hands can stick out of bounding box
    double half_h = h.size_y / 2.0;
    return std::abs(apple.center.position.x - h.center.position.x) < half_w &&
           std::abs(apple.center.position.y - h.center.position.y) < half_h;
}



void FSMNode::switchState(const std::string& name){
    if(states_.find(name) == states_.end()){
        RCLCPP_ERROR(get_logger(), "Unknown state '%s'", name.c_str());
        return;
    }
    pending_ = name;
}

void FSMNode::tick(){
    if (!pending_.empty()){
        if(current_) current_->onExit();
        current_ = states_.at(pending_).get();
        current_name_ = pending_;
        pending_.clear();
        state_entered_ = now();
        current_->onEnter();
    }
    if(current_) current_->execute();
}



int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FSMNode>());
    rclcpp::shutdown();
    return 0;
}