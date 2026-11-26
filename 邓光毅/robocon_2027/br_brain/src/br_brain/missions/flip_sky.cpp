#include "br_brain/missions/flip_sky.h"


#include <iostream>

FlipSky::FlipSky(WorldModel* world_model, SkillLayer* skill_layer, MissionDetails mission_details) :
Mission(world_model, skill_layer, mission_details) {}








void FlipSky::Enter()
{

}










void FlipSky::Stop()
{
    //其余操作待定
    std::cout<<"[任务] FlipSky 任务终止 各机构回归默认安全位(待写)\n";
}


void FlipSky::Arb_Interrupt()
{
    this->Is_Mission_Normally = false;
}





Mission::MissionStatus FlipSky::Execute()
{

    return MissionStatus::Done;

}
