#include "br_brain/missions/fetch_earth.h"


#include <iostream> 

FetchEarth::FetchEarth(WorldModel* world_model, SkillLayer* skill_layer, MissionDetails mission_details) : 
Mission(world_model, skill_layer, mission_details) {}







void FetchEarth::Enter()
{

}









void FetchEarth::Stop()
{   
    //其余操作待定
    std::cout<<"[任务] FetchEarth 任务终止 各机构回归默认安全位(待写)\n";
}



void FetchEarth::Arb_Interrupt()
{
    this->Is_Mission_Normally = false;
}







Mission::MissionStatus FetchEarth::Execute()
{

    return MissionStatus::Done;

}




