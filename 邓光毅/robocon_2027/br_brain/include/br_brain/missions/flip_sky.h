#pragma once

#include "br_brain/missions/mission_base.h"

class FlipSky : public Mission
{
public:
    FlipSky(WorldModel* world_model, SkillLayer* skill_layer, MissionDetails mission_details);

    void Enter() override;
    MissionStatus Execute() override;
    void Stop() override;

    void Arb_Interrupt() override;


private:


};