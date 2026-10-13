#pragma once

#include "br_brain/missions/mission_base.h"




class EnterField : public Mission
{
public:
    EnterField(WorldModel* world_model, SkillLayer* skill_layer);

    MissionID GetMissionID() override;
    void Enter() override;
    MissionStatus Execute() override;
    void Stop() override;

    void Arb_Interrupt() override;



private:
    //原用于测试 现无用
    int tick_count = 0;

    //场地目标点
    double target_x = 0.0;
    double target_y = 0.0;



};