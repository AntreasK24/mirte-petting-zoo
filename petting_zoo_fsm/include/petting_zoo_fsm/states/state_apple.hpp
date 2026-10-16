#pragma once
#include <string>
#include "petting_zoo_fsm/state.hpp"

class AppleState : public State {
public:
    explicit AppleState(FSMNode* fsm);
    void onEnter() override;
    void execute() override;
    void onExit() override;

    std::string arm_target_;   //what the arm follows: "" = human, "liked" = liked apple, "hold" = stay still
};
