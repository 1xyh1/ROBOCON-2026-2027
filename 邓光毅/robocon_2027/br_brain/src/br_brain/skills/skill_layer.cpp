#include "br_brain/skills/skill_layer.h"


//引入各个技能头文件
#include "br_brain/skills/skill_chassis.h"




SkillLayer::SkillLayer(WorldModel* world_model)
{
    //新技能在此注册
    this->skills.push_back(std::make_unique<SkillChassis>(world_model));


}

SkillLayer::~SkillLayer() {}

