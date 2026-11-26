#pragma once

#include <iostream>
#include <vector>
#include <cstdint>
#include <map>

#include "br_brain/defs.h"

/*
暂时准备使用黑板架构 世界模型作为黑板
满足二点
1 世界模型只存当前世界各个信息的状态 不做决策 不依赖其他任何逻辑有关头文件
2 每个/些数据有其唯一的写者 所有人都至少可读






*/









class WorldModel
{
public:
    WorldModel();

    bool safety_flag = true;


    //基础配置信息
    TeamSide team_side = TeamSide::RED; //默认红队







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
    static constexpr uint8_t CELL_STAIR    = 61;    //阶梯
    static constexpr uint8_t CELL_RAMP    = 100;   //斜坡
    static constexpr uint8_t CELL_L0    = 230;   //层
    static constexpr uint8_t CELL_L1    = 204;   //层
    static constexpr uint8_t CELL_L2    = 179;   //层
    static constexpr uint8_t CELL_PUBLIC    = 128;   //公共区域
    static constexpr uint8_t CELL_NON_PUBLIC = 254;  //非公共区域

    int map_width = 0;                      //地图的宽度 列数
    int map_height = 0;                     //地图的高度 行数
    double map_resolution = 0.0;            //分辨率 即每格的大小
    double map_origin_x = 0.0;              //绝对坐标系下 地图的坐标原点
    double map_origin_y = 0.0;              
    std::vector<uint8_t> map_cells;         //存储每个格子的数据 大小为列*行
    std::vector<uint8_t> real_cells;         //存储真实情况的数据 即内置地图和雷达扫描的障碍的数据

    std::vector<MapCell> cells_info;        //格子全部的信息

    std::vector<Transition> transitions;    //上下层通道(拓扑图的边) 由配置加载

    FloorType current_floor = FloorType::NONE;   //当前所在楼层






    //当前场上情况 供任务分配函数读取
    double race_time_left = 180;        //比赛剩余时间
    bool field_entered = false;         //BR进入自身场地与否
    bool F_mustika_condition = false;     //友方圣物放置的条件达成与否
    bool E_mustika_condition = false;     //敌方圣物放置的条件达成与否
    bool Can_getmustika = false;
    //std::vector<BuildSpot> build_spot;
    std::vector<ItemType> carrying_on;  //带在身上的物块
    std::map<Area, SpotInfo> spot_info; //建造点位的信息
    std::map<Area, std::pair<double, double> > area_pose;
    std::map<ItemType, uint8_t> transfer_item;  //交接区当前有的物块


    static constexpr int MAX_CARRYON = 2; 
    void UpdateSpotInfo(Area spot, BuildState state);
    void UpdateTransferInfo(ItemType item, uint8_t num);
    void UpdateFloor(double x, double y);


    //根据坐标返回所属格子
    bool PointAtCell(double x, double y, int& col, int& row);

    
    


    



private:

    //判断格子8邻域内是否有静态墙 用以过滤擦边的点
    bool NearOccupied(int idx);


};