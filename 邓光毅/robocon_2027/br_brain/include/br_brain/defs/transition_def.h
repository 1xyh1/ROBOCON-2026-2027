#pragma once

//用于处理楼梯斜坡等transition区域
#include <cstdint>

#include "br_brain/defs/mapcell_def.h"
#include "br_brain/defs/teamside_def.h"

enum class TransitionKind : uint8_t
{
    None,
    Stair,
    Ramp,
};


//一条上下层通道  两层之间的连接关系直接由配置给出 不再从掩膜推导
//上下行都允许 区别只在于执行时要选不同的特殊动作
struct Transition
{
    int id = -1;
    TransitionKind kind = TransitionKind::Stair;

    FloorType lower = FloorType::NONE;   //低层
    FloorType upper = FloorType::NONE;   //高层

    TeamSide team_side = TeamSide::RED;

    //低层侧/高层侧的粗略入口点  全局规划只需要走到这里 精细路径由导航动作细化
    double lower_x = 0.0;
    double lower_y = 0.0;
    double upper_x = 0.0;
    double upper_y = 0.0;

};
