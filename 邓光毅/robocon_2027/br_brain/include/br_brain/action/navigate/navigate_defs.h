#pragma once

#include <cstdint>
#include <iostream>

#include "br_brain/defs/area_def.h"


//路径区域的类型 是平地还是需要特殊操作的换层区域
enum class NavMode : uint8_t
{
    FLAT,
    TRANSITION,
};



//进入状态机的结构体
struct NavState
{
    size_t id = 0;      //编号 对于其他信息都相同的不同状态 这是唯一区分方式

    NavMode mode = NavMode::FLAT;   //导航状态
    // TransitionKind trans_mode;  //transition的类型

    //该状态需要前往的目标点
    double target_x = 0.0;
    double target_y = 0.0;


    //由于状态机模板中存储状态的是map类型 所以需要重载<号
    bool operator<(const NavState& other) const {return this->id < other.id; }


};





struct NavigateRequire
{
    Area target_area;
    double target_x;
    double target_y;

};




