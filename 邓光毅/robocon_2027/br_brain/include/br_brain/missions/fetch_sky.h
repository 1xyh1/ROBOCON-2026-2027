#pragma once

#include "br_brain/missions/mission_base.h"

class FetchSky : public Mission
{
public:
    FetchSky(WorldModel* world_model, SkillLayer* skill_layer, MissionDetails mission_details);

    void Enter() override;
    MissionStatus Execute() override;
    void Stop() override;

    void Arb_Interrupt() override;


private:


};