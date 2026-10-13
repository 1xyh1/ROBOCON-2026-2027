#include "br_brain/missions/enter_field.h"


#include <iostream>





//技能头文件
#include "br_brain/skills/skill_chassis.h"





EnterField::EnterField(WorldModel* world_model, SkillLayer* skill_layer) : Mission(world_model, skill_layer) {}



MissionID EnterField::GetMissionID()
{
    return MissionID::EnterField;
}







void EnterField::Enter()
{
    this->tick_count = 0;

    //从仲裁层技能池中获取技能指针 push进当前任务的技能池
    this->current_skillpool.push_back(this->skill_layer->Borrow<SkillChassis>());    //底盘控制技能


    //入场目标地点 随便定的
    this->target_x = 2.0;
    this->target_y = -2.0;



    //输出检查
    std::cout<<"[任务] 进入 EnterField 任务 目标点 ("<<this->target_x<<", "<<this->target_y<<") \n";

}




void EnterField::Stop()
{
    //先扇出暂停所有技能
    this->StopAllSkills();




    //其余操作待定
    std::cout<<"[任务] EnterField 任务终止 各机构回归默认安全位(待写)\n";

}



void EnterField::Arb_Interrupt()
{
    this->Is_Mission_Normally = false;
}




Mission::MissionStatus EnterField::Execute()
{

    if(this->Is_Mission_Normally == false)
    {
        std::cout<<"[任务] EnterField 任务异常出错\n";
        return MissionStatus::Failed;
    }

    //测试 底盘走到目标点才会完成任务
    //this->tick_count++;
    SkillChassis* chassis = this->GetCurrentSkill<SkillChassis>();

    if(!chassis)
    {
        std::cout<<"底盘技能指针不存在 任务失败\n";
        return MissionStatus::Failed;
    }


    if(chassis->NavigateTo(this->target_x, this->target_y))
    {
        std::cout<<"[任务] EnterField 任务正常完成\n";
        return MissionStatus::Done;
    }


    return MissionStatus::Running;

} 









