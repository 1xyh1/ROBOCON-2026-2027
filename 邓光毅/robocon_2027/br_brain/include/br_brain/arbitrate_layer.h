#pragma once

#include <memory>
#include <functional>

//引入领域词汇
#include "br_brain/common_defs.h"


#include "br_brain/world_model.h"
#include "br_brain/missions/mission_base.h"


/*

仲裁层用以决断任务 只产生当前的意图 决定现在在做什么 安全吗 该做什么 要不要换任务

仲裁类绝不不触碰执行

维护唯一任务指针 任务中断 完成时均释放内存清空指针



如果处于闲置或初始化状态
由EnterMission函数载入ChooseMission函数分配的任务 
同时EnterMission函数内打印日志 仲裁层交接current_mission任务指针 同时使用SetState函数更新current_state


打断指令发出后 由任务派生类自己返回异常 仲裁层检测到异常终止任务

当外部各种因素导致仲裁需要打断时 仍然走同一套流程 任务自检->返回异常->仲裁层清空任务->回归闲置
确保任何情况下的终止任务都有规范的停止流程 


*/



/*
简要说明
仲裁层只管 事件处理 任务状态机 任务选择策略
任务对象由装配者组装 凭借任务名称ID向任务creator获取指针 
技能由任务在enter时向技能层借 仲裁层不认识任何技能 仲裁层不拥有基类以外的指针

*/

using MissionCreator = std::function<std::unique_ptr<Mission>(MissionID)>;



class ArbitrateLayer
{
public:

    ArbitrateLayer(WorldModel* world_model, MissionCreator mission_creator);

    void ArbitrateUpdate(Event event);              //每周期的更新

private:

    MissionID ChooseMission();                      //任务的分配 与当前的得分策略有关
    bool NeedInterrupt();                           //判断任务是否需要强制中断
    void Interrupt();                               //执行强制中断
    void EnterMission(MissionID id);                //任务分配
    void SetState(MissionID state);                 //将切换运行状态封装为一个函数 便于输出日志


private:

    WorldModel* world_model = nullptr;
    MissionCreator mission_creator;

    std::unique_ptr<Mission> current_mission = nullptr;
    MissionID current_state = MissionID::Init;






    bool match_over = false;
    bool field_entered = false;


};