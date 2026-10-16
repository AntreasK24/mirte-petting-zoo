#include "petting_zoo_fsm/states/state_searching.hpp"
#include "petting_zoo_fsm/fsm_node.hpp"

SearchingState::SearchingState(FSMNode* fsm) : State(fsm) {}

void SearchingState::onEnter() {
    RCLCPP_INFO(fsm_->get_logger(), "Entering Searching State");
    fsm_->setTracking(false);
    fsm_->setWalking(true);
    fsm_->human_visible_ = false;
}

void SearchingState::onExit() {
    RCLCPP_INFO(fsm_->get_logger(), "Exiting Searching State");
}

void SearchingState::execute() {
    if (fsm_->now() < fsm_->ignore_humans_until_) {
        fsm_->human_visible_ = false;   // still in cooldown, ignore detections
        return;
    }
    if (fsm_->human_visible_) fsm_->switchState("tracking");
}