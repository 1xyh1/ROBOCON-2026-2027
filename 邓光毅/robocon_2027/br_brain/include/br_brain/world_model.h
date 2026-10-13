#pragma once

#include <iostream>
#include <vector>
#include <cstdint>



/*
暂时准备使用黑板架构 世界模型作为黑板
满足二点
1 世界模型只存当前世界各个信息的状态 不做决策 不依赖其他任何头文件
2 每个/些数据有其唯一的写者 所有人都至少可读






*/









class WorldModel
{
public:
    WorldModel();

    bool safety_flag = true;

    //暂时先大部分设为公有变量 后续再考虑私有化



    //最终位姿的计算流程
    //最终位姿      由map->odom修正得到 属于绝对坐标系  不直接赋值的原因是amcl估计时帧数有限 不是实时
    double pose_x = 0.0;
    double pose_y = 0.0;
    double pose_yaw = 0.0;  //偏航角[-180, 180]

    //odom原始位姿 可能存在偏移
    double odom_x = 0.0;
    double odom_y = 0.0;
    double odom_yaw = 0.0;

    //map->odom修正边  即odom坐标系相对于map绝对坐标系偏移了多少
    double corr_x = 0.0;
    double corr_y = 0.0;
    double corr_yaw = 0.0;

    



    //用amcl算法修正里程计的漂移误差    手写反推 不使用内置tf2查询 避免耦合各种依赖
    //amcl锚点更新时更新
    void UpdateCorrection(double amcl_x, double amcl_y, double amcl_yaw);
    //每帧更新
    void UpdatePosition();












    //雷达有关部分
    //雷达每帧0.2秒 极坐标形式
    static constexpr int SCAN_BEAM_COUNT = 360;     //constexpr 编译时嵌入机器码 不用运行时赋值 
    double scan_ranges[SCAN_BEAM_COUNT] = {0};
    double scan_angle_min = 0.0;                    //雷达自己坐标系下第一束波的起始角度 即相对雷达传感器零方向的偏差
    double scan_angle_forward = 0.0;                //雷达零方向在绝对坐标系下的朝向 = pose_yaw + 雷达安装偏角
    double scan_angle_increment = 0.0;              //波与波之间的角度差


    //雷达扇区检测 如果前进方向有物体立马停止 返回传感器系下 [center-half, center+half] 角度区间内最近的回波距离
    double MinRangeInSector(double center_angle, double half_width);

    //地图的动态更新层
    //当前障碍的新鲜度即时间戳由dyn_stamp判定 时刻基准是 scan_stamp_s 由雷达的时间系确定
    static constexpr double DYN_TTL = 1.0;               //障碍新鲜度持续时间 超过清除地图中障碍


    std::vector<uint8_t> dyn_cells;     //0=无标记 1=有障碍
    std::vector<double> dyn_stamp;      //每个格子最近一次标记的时刻
    size_t dyn_version = 0;             //检查到每次动态障碍变化后认为是一个新世纪

    double scan_stamp_s = 0.0;          //最后一帧雷达的时间戳
    void UpdateDynamicLayer();
    








    //与ros2其他节点沟通的命令集

    //底盘速度
    double cmd_v = 0.0;     //  m/s
    double cmd_w = 0.0;     //  rad/s 角速度 逆时针为正








    //场地固有属性

    //场地地图 由tools工具函数(文件)map_loader加载
    //灰度图的阈值决定了可走与否 0-255共256个数 故用uint8
    static constexpr uint8_t CELL_OCCUPIED = 0;     //墙
    static constexpr uint8_t CELL_FREE     = 254;   //可走
    static constexpr uint8_t CELL_UNKNOWN  = 205;   //未知

    int map_width = 0;                      //地图的宽度 列数
    int map_height = 0;                     //地图的高度 行数
    double map_resolution = 0.0;            //分辨率 即每格的大小
    double map_origin_x = 0.0;              //绝对坐标系下 地图的坐标原点
    double map_origin_y = 0.0;              
    std::vector<uint8_t> map_cells;         //存储每个格子的数据 大小为列*行





    
    


    



private:

    //判断格子8邻域内是否有静态墙 用以过滤擦边的点
    bool NearOccupied(int idx);


};