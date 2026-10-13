#include "br_brain/arbitrate_layer.h"



//匿名命名空间 仅当前cpp可见
namespace
{
    const char* StateName(MissionID state)
    {
        switch(state)
        {
            case MissionID::Init: return "Init";
            case MissionID::Idle: return "Idle";
            case MissionID::BuildEarth: return "BuildEarth";
            case MissionID::CapSky: return "CapSky";
            case MissionID::FlipSky: return "FlipSky";
            case MissionID::PlaceOrb: return "PlaceOrb";
            case MissionID::EnterField: return "EnterField";
            case MissionID::GoRetryField: return "GoRetryField";
            
        }

        return "Unknown";

        //便于打印日志

    }
}





//初始化 
ArbitrateLayer::ArbitrateLayer(WorldModel* world_model, MissionCreator mission_creator)
 : world_model(world_model), mission_creator(mission_creator) {}






void ArbitrateLayer::SetState(MissionID state)
{
    if(this->current_state == state)return;

    std::cout<<"[仲裁] 任务状态切换: "<<StateName(this->current_state) <<"->" <<StateName(state) <<"\n";


    this->current_state = state;
}











void ArbitrateLayer::ArbitrateUpdate(Event event)
{

    //第一步安全检查
    if(this->world_model->safety_flag == false )
    {
        std::cout<<"[仲裁] 安全检查不通过\n";
        if(this->current_mission)   //有任务即打断 Init及Idle无任务
        {
            std::cout<<"[仲裁] 由于安全问题 将执行打断指令";
            this->Interrupt();
        }
        
    }

    //第二步判断事件是否需要打断施法并且闲置待机
    if(event == Event::RefereeRetry || event == Event::Buzzer)
    {
        
        if(event == Event::RefereeRetry)
        {
            std::cout<<"[仲裁] 收到RefereeRetry事件 \n";
        }

        if(event == Event::Buzzer)
        {
            std::cout<<"[仲裁] 收到Buzzer事件 \n";
            this->match_over = true;
        } 


        if(this->current_state != MissionID::Idle)
        {
            this->Interrupt();
        }
        
    }


    //当前的运行任务状态
    switch(this->current_state)
    {
        case MissionID::Init :
            //开赛信号
            if(event == Event::Start)
            {
                this->EnterMission(this->ChooseMission());
                
            }

            break;
        
        case MissionID::Idle ://闲置的时候分配任务
        {
            if(this->match_over || this->world_model->safety_flag == false) break;
            
            //直接分配任务    
            this->EnterMission(this->ChooseMission());
    
            break;
        }               //此处大括号的目的是单独开一块作用域 防止ms暴露在switch的整个作用域下 从而避免在未初始化时析构释放导致崩溃

        case MissionID::EnterField :
        {

            if(this->NeedInterrupt())
            {
                this->Interrupt();
                
            }
            

            Mission::MissionStatus mission_status = this->current_mission->Execute();
            if(mission_status != Mission::MissionStatus::Running)     //在任务执行的同时进行任务执行状态的对齐
            {
                this->current_mission->Stop();
                this->current_mission.reset();  //释放唯一指针
                if(mission_status == Mission::MissionStatus::Done)
                {
                    this->field_entered = true;
                    this->SetState(MissionID::Idle);
                }

                if(mission_status == Mission::MissionStatus::Failed)
                {
                    //任务失败后恢复默认状态
                    this->SetState(MissionID::Idle);
                }


                break;
            }


            break;
        }

        case MissionID::BuildEarth :
            break;


        case MissionID::CapSky :

            break;


        case MissionID::FlipSky :

            break;


        case MissionID::PlaceOrb :

            break;

    }

    return;
}










void ArbitrateLayer::EnterMission(MissionID id)
{
    std::unique_ptr<Mission> mission = this->mission_creator(id);
    if(!mission)
    {
        std::cout<<"[Err] 任务无法正常分配\n";
        return;
    }

    this->current_mission = std::move(mission);
    this->current_mission->Enter();                 //任务进入 包含技能树的初始化
    this->SetState(this->current_mission->GetMissionID());
    return;
}







bool ArbitrateLayer::NeedInterrupt()
{


    return false;
}


void ArbitrateLayer::Interrupt()
{
    if(this->current_mission)
    {
        std::cout<<"[仲裁] 将打断当前 "<<StateName(this->current_state)<<" 任务\n";
        // this->current_mission->Stop();
        // this->current_mission.reset();      //重置释放该智能指针 防止内存溢出
        this->current_mission->Arb_Interrupt();     //现改为仅向任务派生类发送打断指令 由其调用自身函数确认任务失败
    }
    else    //如果指针异常为空 则直接回归闲置状态
    {
        this->SetState(MissionID::Idle);
    } 

    return;


}



MissionID ArbitrateLayer::ChooseMission()
{
    if(this->field_entered == false)
        return MissionID::EnterField;



    return MissionID::BuildEarth;
}