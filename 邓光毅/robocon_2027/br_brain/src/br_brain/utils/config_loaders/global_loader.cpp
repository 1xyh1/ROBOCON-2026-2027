#include "br_brain/utils/config_loader.h"

#include <filesystem>
#include <yaml-cpp/yaml.h>



bool LoadGlobalConfig(const std::string& path, ConfigPaths& out)
{
    //确定根路径    br_brain/config
    const std::filesystem::path base = std::filesystem::path(path).parent_path();

    try
    {
        YAML::Node global_cfg = YAML::LoadFile(path);
    
        out.field = (base / global_cfg["configs"]["field"].as<std::string>()).string();
        out.sensor = (base / global_cfg["configs"]["sensor"].as<std::string>()).string();
        out.map = (base / global_cfg["configs"]["map"].as<std::string>()).string();
        out.mask1 = (base / global_cfg["configs"]["mask1"].as<std::string>()).string();
        out.mask2 = (base / global_cfg["configs"]["mask2"].as<std::string>()).string();
        out.mask3 = (base / global_cfg["configs"]["mask3"].as<std::string>()).string();

        

        return true;
    }
    catch(const YAML::Exception& e)
    {
        std::cerr << e.what() << "\n";
        std::cout<<"[配置] 读取全局配置路径出错\n";
        return false;
    }
    

}

bool LoadALL(const std::string& path, WorldModel& wm)
{
    ConfigPaths cfg;
    //若读取配置路径出错则直接false
    if(!LoadGlobalConfig(path, cfg))
    {
        return false;
    }

    bool is_all_ok = true;

    if(!LoadFieldConfig(cfg.field, wm)) is_all_ok = false;
    // if(!LoadSensorConfig(cfg.sensor, wm)) is_all_ok = false;
    if(!LoadMap(cfg.map, wm)) is_all_ok = false;
    if(!LoadStructureMask(cfg.mask1, wm)) is_all_ok = false;
    if(!LoadFloorMask(cfg.mask2, wm)) is_all_ok = false;
    if(!LoadRegionMask(cfg.mask3, wm)) is_all_ok = false;


    if(!is_all_ok)
    {
        std::cout<<"[配置] 有配置无法正常加载\n";
        return false;
    }

    return true;

}