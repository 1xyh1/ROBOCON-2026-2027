#include "br_brain/action/action_sequence.h"



ActionSequence::ActionSequence() {}


void ActionSequence::StopActions()
{
    for(auto& action : this->actions)
    {
        action->StopAction();
    }
}




void ActionSequence::AddAction(std::unique_ptr<Action> action)
{
    //将动作指针转移
    if(action)
    {
        this->actions.push_back(std::move(action)); 
        this->actions_size++;

    }
    else
    {
        std::cout<<"[动作] 动作注册出现错误\n";
        return;
    }
}



ActionState ActionSequence::Execute()
{
    // if(this->finished) return ActionState::DONE;

    if(!this->started)
    {
        this->actions[this->index]->Start();
        this->started = true;
        // std::cout<<"[Action] 动作开始\n";
    }

    
    ActionState state = this->actions[this->index]->Execute();

    if(state == ActionState::DONE)
    {
        this->index++;
        this->started = false;

        if(this->index >= this->actions_size)
        {
            finished = true;
            std::cout<<"[Action] 动作链全部结束\n";
            return ActionState::DONE;
        }
    }
    else if(state == ActionState::FAILED)
    {
        this->failed = true;
        std::cout<<"[Action] 动作失败\n";

        return ActionState::FAILED;
        //暂时没有决定好失败之后要做什么
    }

    return ActionState::RUNNING;


}






