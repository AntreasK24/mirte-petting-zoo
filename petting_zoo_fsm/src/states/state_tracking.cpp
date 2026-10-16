#include "petting_zoo_fsm/states/state_tracking.hpp"
#include "petting_zoo_fsm/fsm_node.hpp"

TrackingState::TrackingState(FSMNode* fsm) : State(fsm) {}

void TrackingState::onEnter() {
    RCLCPP_INFO(fsm_->get_logger(), "Entering Tracking State");
    fsm_->setWalking(false);
    fsm_->setTracking(true);
    fsm_->apple_in_hand_ = false;
}

void TrackingState::onExit() {
    RCLCPP_INFO(fsm_->get_logger(), "Exiting Tracking State");
}

void TrackingState::execute() {
    double since_seen = (fsm_->now() - fsm_->last_seen_).seconds();
    double in_state   = (fsm_->now() - fsm_->state_entered_).seconds();
    if (fsm_->apple_in_hand_) { fsm_->switchState("apple"); return; }
    if (in_state > fsm_->max_track_time_) {
        RCLCPP_INFO(fsm_->get_logger(), "Tracked for %.0f s, moving on", in_state);
        fsm_->ignore_humans_until_ = fsm_->now() + rclcpp::Duration::from_seconds(fsm_->cooldown_);
        fsm_->switchState("searching");
    } else if (since_seen > fsm_->lost_timeout_) {
        RCLCPP_INFO(fsm_->get_logger(), "Lost human for %.0f s", since_seen);
        fsm_->switchState("searching");
    }
}