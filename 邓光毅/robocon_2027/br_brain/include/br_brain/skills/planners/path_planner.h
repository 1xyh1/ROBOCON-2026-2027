#pragma once


#include "br_brain/world_model.h"


#include <vector>
#include <utility>
#include <string>



/*

技能类包含各种规划器

路径规划器 架构上认为是世界模型在路径规划处理上的内置工具 故直接耦合世界模型

路径规划仅对路径和膨胀地图作用 不参与其他决策



*/





class PathPlanner
{
public:
    PathPlanner(WorldModel* world_model);

    //规划出来的路径 依次走的绝对坐标
    std::vector<std::pair<double, double> > out_path;

    //判断当前规划好的路径在最新动态地图上是否仍然可行
    bool IsPathValid();



    //生成膨胀图
    bool InflateMap();


    //A*寻路 输入膨胀图 输出路径坐标点列
    bool PlanPath_AStar(double now_x, double now_y, double target_x, double target_y);

    //路线剪枝
    void SimplifyPath();

    //调试输出: 膨胀图+路径画成pgm图片 肉眼验收一次性用
    void DumpDebugPgm(const std::vector<std::pair<double, double> >& path, const std::string& file_path);


private:

    //世界点坐标转为地图格子下标 其为地图坐标系 越界返回false      传入引用 直接赋值
    bool WorldToCell(double x, double y, int& col, int& row);

    //计算曼哈顿距离 用于A*估计代价     res是地图的分辨率 每格的m数 用以计算真实距离代价
    int ManhattanDistance(int n_idx, int t_idx, int width);


    //LOS 视线检测 判断两点之间是否完全无阻碍 用于路径平滑
    bool HasLineOfSight(double x0, double y0, double x1, double y1);



private:

    WorldModel* world_model;                //路径规划依赖于世界模型 故存储世界模型指针

    std::vector<uint8_t> inflated_cells;    //膨胀图 墙长出机器人半径  用于A*寻路


    //地图膨胀函数 预计膨胀的尺寸为 0.3m
    static constexpr double InflateRadius = 0.3;



};