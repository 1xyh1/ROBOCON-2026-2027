#include "br_brain/missions/mission_base.h"

Mission::Mission(WorldModel* world_model, SkillLayer* skill_layer) : world_model(world_model), skill_layer(skill_layer){}




void Mission::StopAllSkills()
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

