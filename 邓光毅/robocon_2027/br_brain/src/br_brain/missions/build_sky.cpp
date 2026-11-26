#include "br_brain/missions/build_sky.h"


#include <iostream>

BuildSky::BuildSky(WorldModel* world_model, SkillLayer* skill_layer, MissionDetails mission_details) :
Mission(world_model, skill_layer, mission_details) {}








void BuildSky::Enter()
{

}










void BuildSky::Stop()
{
    //其余操作待定
    std::cout<<"[任务] BuildSky 任务终止 各机构回归默认安全位(待写)\n";
}


void BuildSky::Arb_Interrupt()
{
    this->Is_Mission_Normally = false;
}





Mission::MissionStatus BuildSky::Execute()
{

    return MissionStatus::Done;

}
