#pragma once

#include "br_brain/world_model.h"
#include <utility>
//规划全局宏观路径
//根据当前坐标和目标坐标给出大致路径节点
//x,y
using Point = std::pair<double, double>;


struct RouteStep
{
    Point target;           //先走到目标点
    int transition_id = -1; //然后走该id对应的transition

};


class RoutePlanner
{
public:


    //规划器给定起点和终点 规划出大致要经过的点
    bool Plan(WorldModel* world_model, Point begin, Point end);



    std::vector<RouteStep> out_path;




private:



    


};
