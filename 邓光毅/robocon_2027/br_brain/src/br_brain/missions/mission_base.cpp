#include "br_brain/missions/mission_base.h"

Mission::Mission(WorldModel* world_model, SkillLayer* skill_layer, MissionDetails mission_details) : 
world_model(world_model), skill_layer(skill_layer), mission_details(mission_details) 
{
    this->action_sequnce = std::make_unique<ActionSequence>();

}



MissionID Mission::GetMissionID()
{
    return this->mission_details.mission_id;
}





