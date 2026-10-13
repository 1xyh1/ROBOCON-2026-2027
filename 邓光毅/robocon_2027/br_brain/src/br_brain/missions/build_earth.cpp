#include "br_brain/missions/build_earth.h"


#include <iostream> //虽然最顶层的头文件有使用  但是秉持哪里用就include的原则写 避免更改顶层文件导致下层崩溃

BuildEarth::BuildEarth(WorldModel* world_model, SkillLayer* skill_layer) : Mission(world_model, skill_layer) {}



MissionID BuildEarth::GetMissionID()
{
    return MissionID::BuildEarth;
}








void BuildEarth::Enter()
{

}









void BuildEarth::Stop()
{   
    //其余操作待定
    std::cout<<"[任务] BuildEarth 任务终止 各机构回归默认安全位(待写)\n";
}



void BuildEarth::Arb_Interrupt()
{
    this->Is_Mission_Normally = false;
}







Mission::MissionStatus BuildEarth::Execute()
{

    if(this->Is_Mission_Normally == false)
    {
        std::cout<<"[任务] BuildEarth 任务异常出错\n";
        return MissionStatus::Failed;
    }

    //测试 假设经过400个周期完成 即4秒
    this->tick_count++;
    if(this->tick_count >= 400)
    {
        std::cout<<"[任务] BuildEarth 任务正常完成\n";
        return MissionStatus::Done;
    }


    return MissionStatus::Running;

}




