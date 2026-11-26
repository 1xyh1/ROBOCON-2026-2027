#include "br_brain/missions/fetch_mustika.h"


#include <iostream>

FetchMustika::FetchMustika(WorldModel* world_model, SkillLayer* skill_layer, MissionDetails mission_details) :
Mission(world_model, skill_layer, mission_details) {}








void FetchMustika::Enter()
{

}










void FetchMustika::Stop()
{
    //其余操作待定
    std::cout<<"[任务] FetchMustika 任务终止 各机构回归默认安全位(待写)\n";
}


void FetchMustika::Arb_Interrupt()
{
    this->Is_Mission_Normally = false;
}





Mission::MissionStatus FetchMustika::Execute()
{

    return MissionStatus::Done;

}
