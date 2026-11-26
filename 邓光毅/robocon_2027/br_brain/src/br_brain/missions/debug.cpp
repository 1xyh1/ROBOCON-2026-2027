#include "br_brain/missions/debug.h"


#include <iostream>

Debug::Debug(WorldModel* world_model, SkillLayer* skill_layer, MissionDetails mission_details) :
Mission(world_model, skill_layer, mission_details) {}





void Debug::Enter()
{

}





void Debug::Stop()
{
    //其余操作待定
    std::cout<<"[调试任务] 调试 任务终止 各机构回归默认安全位(待写)\n";
}


void Debug::Arb_Interrupt()
{
    this->Is_Mission_Normally = false;
}





Mission::MissionStatus Debug::Execute()
{
    cnt++;

    if(cnt>=200)
    {

        std::cout<<"[调试任务] debugging任务完成\n";
        return MissionStatus::Done;

    }



    return MissionStatus::Running;


}
