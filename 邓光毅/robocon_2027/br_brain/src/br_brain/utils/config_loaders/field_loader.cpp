#include "br_brain/utils/config_loader.h"
#include "br_brain/world_model.h"


#include <filesystem>
#include <yaml-cpp/yaml.h>




static bool ParseFloor(const std::string& s, FloorType& out)
{
    if(s == "L0") { out = FloorType::L0; return true; }
    if(s == "L1") { out = FloorType::L1; return true; }
    if(s == "L2") { out = FloorType::L2; return true; }
    return false;
}

static bool ParseKind(const std::string& s, TransitionKind& out)
{
    if(s == "stair") { out = TransitionKind::Stair; return true; }
    if(s == "ramp")  { out = TransitionKind::Ramp;  return true; }
    return false;
}









bool LoadFieldConfig(const std::string& path, WorldModel& wm)
{
    try
    {
        YAML::Node field_cfg = YAML::LoadFile(path);
        
        if(field_cfg["race"]["team_side"].as<std::string>() == "red")
        {
            wm.team_side = TeamSide::RED;
            std::cout<<"[配置] 加载field_config成功 队伍为RED\n";
        }
        else if(field_cfg["race"]["team_side"].as<std::string>() == "blue")
        {
            wm.team_side = TeamSide::BLUE;
            std::cout<<"[配置] 加载field_config成功 队伍为BLUE\n";
        }

        if(field_cfg["transitions"])
        {
            for(const auto& n : field_cfg["transitions"])
            {
                Transition t;
                t.id    = n["id"].as<int>();
            
                if(!ParseKind(n["kind"].as<std::string>(), t.kind) ||
                !ParseFloor(n["lower"].as<std::string>(), t.lower) ||
                !ParseFloor(n["upper"].as<std::string>(), t.upper))
                {
                    std::cout << "[配置] transition id=" << t.id
                            << " 的 kind/lower/upper 非法\n";
                    return false;
                }                



                t.lower_x = n["lower_pose"][0].as<double>();
                t.lower_y = n["lower_pose"][1].as<double>();
                t.upper_x = n["upper_pose"][0].as<double>();
                t.upper_y = n["upper_pose"][1].as<double>();
                
                if(t.lower_y > 0) t.team_side = TeamSide::BLUE;
                else t.team_side = TeamSide::RED;

                wm.transitions.push_back(t);
            }
            std::cout << "[配置] transitions: " << wm.transitions.size() << " 条\n";

            for(const auto& t : wm.transitions)
            {
                std::cout << "  id=" << t.id
                        << " kind=" << (t.kind == TransitionKind::Ramp ? "ramp" : "stair")
                        << " " << static_cast<int>(t.lower)
                        << "->" << static_cast<int>(t.upper)
                        << " lower(" << t.lower_x << "," << t.lower_y << ")"
                        << " upper(" << t.upper_x << "," << t.upper_y << ")\n";
            }

            

        }        


        return true;
    }
    catch(const YAML::Exception& e)
    {
        std::cerr << e.what() << "\n";
        std::cout<<"[配置] 读取field_config配置路径出错\n";
        return false;
    }
    


    

}
