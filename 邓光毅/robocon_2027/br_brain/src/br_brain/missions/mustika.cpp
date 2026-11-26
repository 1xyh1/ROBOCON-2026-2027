#include "br_brain/missions/mustika.h"


#include <iostream>

Mustika::Mustika(WorldModel* world_model, SkillLayer* skill_layer, MissionDetails mission_details) :
Mission(world_model, skill_layer, mission_details) {}








void Mustika::Enter()
{

}










void Mustika::Stop()
{
    //其余操作待定
    std::cout<<"[任务] Mustika 任务终止 各机构回归默认安全位(待写)\n";
}


void Mustika::Arb_Interrupt()
{
    this->Is_Mission_Normally = false;
}





Mission::MissionStatus Mustika::Execute()
{

    return MissionStatus::Done;

}
