#pragma once


#include <iostream>
#include <string>

//暂时先分场地与传感器的全局配置 之后考虑细分
//各个load方法单开一个文件实现

class WorldModel;

struct ConfigPaths
{
    std::string field = "";
    std::string sensor = "";
    std::string map = "";
    std::string mask1 = "";
    std::string mask2 = "";
    std::string mask3 = "";
};

//此处传入global.yaml 获取各个配置路径后全部配置
bool LoadALL(const std::string& path, WorldModel& wm);

//根据全局配置获取所有配置文件的路径
bool LoadGlobalConfig(const std::string& path, ConfigPaths& out);


bool LoadFieldConfig(const std::string& path, WorldModel& wm);
bool LoadSensorConfig(const std::string& path, WorldModel& wm);
bool LoadMap(const std::string& path, WorldModel& wm);
bool LoadRegionMask(const std::string& path, WorldModel& wm);
bool LoadStructureMask(const std::string& path, WorldModel& wm);
bool LoadFloorMask(const std::string& path, WorldModel& wm);

