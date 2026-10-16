#pragma once
#include "petting_zoo_fsm/state.hpp"

class TrackingState : public State {
public:
    explicit TrackingState(FSMNode* fsm);
    void onEnter() override;
    void execute() override;
    void onExit() override;
};
