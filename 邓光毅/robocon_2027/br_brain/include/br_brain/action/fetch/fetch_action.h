#pragma once

/*

处理抓取动作 抓取地块 天块 圣物的公共动作


*/




#include "br_brain/action/action.h"
#include "br_brain/action/fetch/fetch_defs.h"





class FetchAction : public Action 
{
public:

    FetchAction(WorldModel* world_model, SkillLayer* skill_layer, FetchRequire require);

    void Start() override;
    ActionState Execute() override;









private:

    //抓取动作的要求
    FetchRequire require;




};



