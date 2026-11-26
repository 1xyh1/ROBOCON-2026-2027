#include "br_brain/action/action.h"



Action::Action(WorldModel* world_model) : world_model(world_model) {}



void Action::StopAction()
{

    for(auto& skill : this->current_skillpool)
    {
        if(skill)
        {
            skill->Stop();
        }
        else
        {
            std::cout<<"[Error] 释放技能空指针\n";
        }
    }


}