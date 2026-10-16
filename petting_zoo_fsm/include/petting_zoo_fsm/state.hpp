#pragma once

class FSMNode;

class State{
    public: 
        explicit State(FSMNode* fsm) : fsm_(fsm) {}
        virtual ~State() = default;

        virtual void onEnter() {}
        virtual void execute() = 0;
        virtual void onExit() {}


    protected:
        FSMNode* fsm_;
};