#include "petting_zoo_fsm/states/state_apple.hpp"
#include "petting_zoo_fsm/fsm_node.hpp"

AppleState::AppleState(FSMNode* fsm) : State(fsm) {}

void AppleState::onEnter() {
    RCLCPP_INFO(fsm_->get_logger(), "Entering Apple State");
    fsm_->setWalking(false);
    fsm_->setTracking(true);    //keep the arm (camera) on the human until the apple is steady
    fsm_->setGripper(true);     //open the gripper
    arm_target_ = "";
}

void AppleState::onExit() {
    RCLCPP_INFO(fsm_->get_logger(), "Exiting Apple State");
    fsm_->setAppleTarget("");      //arm goes back to following humans
    fsm_->setGripper(false);       //close the gripper
}

void AppleState::execute() {
    if (fsm_->dropping_) return;   //no state change or arm target change until the apple is dropped

    double since_human = (fsm_->now() - fsm_->last_seen_).seconds();
    double since_apple = (fsm_->now() - fsm_->apple_last_seen_).seconds();

    if (since_human > fsm_->lost_timeout_) {
        RCLCPP_INFO(fsm_->get_logger(), "Lost human for %.0f s", since_human);
        fsm_->switchState("searching");
    } else if (since_apple > fsm_->apple_lost_timeout_) {
        RCLCPP_INFO(fsm_->get_logger(), "No apple for %.0f s", since_apple);
        fsm_->switchState("tracking");
    } else {
        //follow a steady liked apple, hold the arm still for a steady disliked one (liked wins if both)
        double since_liked    = (fsm_->now() - fsm_->liked_last_seen_).seconds();
        double since_disliked = (fsm_->now() - fsm_->disliked_last_seen_).seconds();
        bool liked_steady    = since_liked    < 0.5 && fsm_->liked_streak_    >= fsm_->apple_track_frames_;
        bool disliked_steady = since_disliked < 0.5 && fsm_->disliked_streak_ >= fsm_->apple_track_frames_;

        std::string target = arm_target_;
        if (liked_steady)         target = "liked";
        else if (disliked_steady) target = "hold";

        if (target != arm_target_) {
            RCLCPP_INFO(fsm_->get_logger(), "%s", target == "liked" ? "Liked apple steady, tracking it"
                                                                    : "Disliked apple steady, holding the arm still");
            fsm_->setAppleTarget(target);
            arm_target_ = target;
        }
    }
}