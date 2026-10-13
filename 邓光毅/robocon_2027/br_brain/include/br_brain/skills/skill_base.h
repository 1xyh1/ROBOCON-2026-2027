#pragma once


#include "br_brain/world_model.h"



#include <memory>

/*

技能基类 与Mission基类同构 基类只统一生命周期

技能与世界交互 因此直接关联世界模型
只把输出写进WorldModel 不碰ROS

*/
class Skill
{
public:
    Skill(WorldModel* world_model);
    virtual ~Skill() = default;


    //技能的安全停止
    virtual void Stop() = 0;





protected:

    WorldModel* world_model;








private:










};