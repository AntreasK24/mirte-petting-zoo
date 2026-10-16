#pragma once
#include "petting_zoo_fsm/state.hpp"

class SearchingState : public State {
public:
    explicit SearchingState(FSMNode* fsm);
    void onEnter() override;
    void execute() override;
    void onExit() override;
};
