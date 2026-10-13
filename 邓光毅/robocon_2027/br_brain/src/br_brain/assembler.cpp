#include "br_brain/assembler.h"


std::unique_ptr<Mission> Assembler::CreateMission(WorldModel* world_model, SkillLayer* skill_layer, MissionID id)
{
    switch(id)
    {
        case MissionID::EnterField : return std::make_unique<EnterField>(world_model, skill_layer);
        case MissionID::BuildEarth : return std::make_unique<BuildEarth>(world_model, skill_layer);

        default : return nullptr;
    }


}


void Assembler::Assemble(std::unique_ptr<WorldModel>& out_wm,
                         std::unique_ptr<SkillLayer>& out_skills,
                         std::unique_ptr<ArbitrateLayer>& out_arb,
                         const std::string& map_yaml_path)
{

    //世界模型
    out_wm = std::make_unique<WorldModel>();
    if(!LoadMap(map_yaml_path, *out_wm))
    {
        std::cout << "[组装] 地图加载失败 使用内置空旷地图(降级运行)\n";
    }

    //技能层
    out_skills = std::make_unique<SkillLayer>(out_wm.get());

    //任务装配回调 匿名函数
    MissionCreator creator = [wm = out_wm.get(), sl = out_skills.get()](MissionID id)->std::unique_ptr<Mission>
    {
        return Assembler::CreateMission(wm, sl, id);
    };


    

    //仲裁层
    out_arb = std::make_unique<ArbitrateLayer>(out_wm.get(), creator);

}