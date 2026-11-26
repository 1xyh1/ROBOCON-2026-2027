#include "br_brain/missions/fetch_sky.h"


#include <iostream>

FetchSky::FetchSky(WorldModel* world_model, SkillLayer* skill_layer, MissionDetails mission_details) :
Mission(world_model, skill_layer, mission_details) {}








void FetchSky::Enter()
{

}










void FetchSky::Stop()
{
    //其余操作待定
    std::cout<<"[任务] FetchSky 任务终止 各机构回归默认安全位(待写)\n";
}


void FetchSky::Arb_Interrupt()
{
    this->Is_Mission_Normally = false;
}





Mission::MissionStatus FetchSky::Execute()
{

    return MissionStatus::Done;

}
