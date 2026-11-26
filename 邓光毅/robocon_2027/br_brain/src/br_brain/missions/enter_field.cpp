#include "br_brain/missions/enter_field.h"


#include <iostream>







#include "br_brain/action/navigate/navigate_action.h"



EnterField::EnterField(WorldModel* world_model, SkillLayer* skill_layer, MissionDetails mission_details) : 
Mission(world_model, skill_layer, mission_details) {}








void EnterField::Enter()
{
    NavigateRequire navi_require = {mission_details.target_area, mission_details.target_x, mission_details.target_y};

    //注册导航动作
    this->action_sequnce->AddAction(std::make_unique<NavigateAction>(world_model, skill_layer, navi_require));


}




void EnterField::Stop()
{
    this->action_sequnce->StopActions();



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

    ActionState state = this->action_sequnce->Execute();

    switch(state)
    {
        case ActionState::DONE : 
            this->world_model->field_entered = true; 
            return MissionStatus::Done;
        case ActionState::FAILED : return MissionStatus::Failed;

        default : return MissionStatus::Running;
    }



} 









