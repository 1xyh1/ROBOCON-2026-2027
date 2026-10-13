#pragma once

#include "br_brain/missions/mission_base.h"

class BuildEarth : public Mission
{
public:
    BuildEarth(WorldModel* world_model, SkillLayer* skill_layer);

    MissionID GetMissionID() override;
    void Enter() override;
    MissionStatus Execute() override;
    void Stop() override;

    void Arb_Interrupt() override;


private:
    int tick_count = 0;


};