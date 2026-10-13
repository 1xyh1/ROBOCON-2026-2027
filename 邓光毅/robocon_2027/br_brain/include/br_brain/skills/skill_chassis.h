#pragma once

#include "br_brain/skills/skill_base.h"

#include "br_brain/skills/planners/path_planner.h"



#include <vector>

/*

技能层
底盘

技能会与世界交互 因此直接关联世界模型类

只负责把速度指令写进WorldModel 不碰ROS


底盘调用雷达数据判断是否要急刹
底盘调用雷达数据获取动态物体信息 并将其可能的路径全部标记为障碍 并设置其存在时间


*/


class SkillChassis : public Skill
{
public:

    SkillChassis(WorldModel* world_model);

    
    void Stop() override;
    
    bool NavigateTo(double target_x, double target_y);    
    bool GoToPoint(double target_x, double target_y);   //true = 到达



    //公有
    //底盘控制技能有路径规划器
    std::unique_ptr<PathPlanner> path_planner = nullptr;

private:

    //反射层急刹函数 判断要不要急刹
    bool Reflex_StopForwardVel();

    bool Need_RePlanned();



private:

    //反射层刹停状态
    bool reflex_stopped = false;

    //是否规划好路径
    bool is_path_planned = false;

    //当前走到的路径的某个点
    int current_point = 0;


    //前一个目标点
    double last_target_x = 0.0;
    double last_target_y = 0.0;

    //前一个动态图更新世纪
    size_t last_dyn_version = -1;   //-1表示暂未开始更新



};