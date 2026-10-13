#pragma once 


#include <iostream>
#include <cstdint>
#include <string>




class WorldModel;   


//读nav2格式地图(pgm+yaml) 阈值归一化后写入WorldModel 成功返回true
//纯C++文件IO 不碰ROS
bool LoadMap(const std::string& yaml_path, WorldModel& wm);

//查询场地坐标(x,y)处的格子值 地图无效或越界返回CELL_UNKNOWN
uint8_t CellAt(const WorldModel& wm, double x, double y);
