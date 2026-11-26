#pragma once



/*

导航动作派生




*/


#include "br_brain/action/action.h"
#include "br_brain/action/navigate/route_planner.h"
#include "br_brain/action/navigate/navigate_defs.h"

#include "br_brain/utils/fsm_base.h"








class NavigateAction : public Action
{
public:

    NavigateAction(WorldModel* world_model, SkillLayer* skill_layer, NavigateRequire require);

    void Start() override;
    ActionState Execute() override;




private:

    NavigateRequire require;

    RoutePlanner route_planner;
    size_t step_idx = 0;

    //假设现在有一个全局规划器 给出了从当前点位到最终目标点位所有要经过的区域 
    //如L0 Transition(楼梯或斜坡) L1 ...
    //那么可以将其输出的信息存入NavState 定义状态机

    FSM<NavState> nav_fsm;
    size_t final_state_id = 0;

};










//作为试验 先将全局规划器放在此处


