#include "br_brain/assembler.h"


std::unique_ptr<Mission> Assembler::CreateMission(WorldModel* world_model, SkillLayer* skill_layer, MissionDetails mission_details)
{
    MissionID id = mission_details.mission_id;


    switch(id)
    {
        case MissionID::EnterField : return std::make_unique<EnterField>(world_model, skill_layer, mission_details);
        case MissionID::FetchEarth : return std::make_unique<FetchEarth>(world_model, skill_layer, mission_details);
        case MissionID::FetchSky : return std::make_unique<FetchSky>(world_model, skill_layer, mission_details);
        case MissionID::FetchMustika : return std::make_unique<FetchMustika>(world_model, skill_layer, mission_details);
        
        
        case MissionID::BuildEarth : return std::make_unique<BuildEarth>(world_model, skill_layer, mission_details);
        case MissionID::BuildSky : return std::make_unique<BuildSky>(world_model, skill_layer, mission_details);
        case MissionID::FlipSky : return std::make_unique<FlipSky>(world_model, skill_layer, mission_details);
        case MissionID::Mustika : return std::make_unique<Mustika>(world_model, skill_layer, mission_details);
        case MissionID::Debug : return std::make_unique<Debug>(world_model, skill_layer, mission_details);


        default : return nullptr;
    }


}


void Assembler::Assemble(std::unique_ptr<WorldModel>& out_wm,
                         std::unique_ptr<SkillLayer>& out_skills,
                         std::unique_ptr<ArbitrateLayer>& out_arb,
                         const std::string& global_yaml_path)
{

    //世界模型
    out_wm = std::make_unique<WorldModel>();
    if(!LoadALL(global_yaml_path, *out_wm))
    {
        std::cout << "[组装] 全局配置加载未全部成功 部分配置将采用默认情况处理\n";
    }

    //技能层
    out_skills = std::make_unique<SkillLayer>(out_wm.get());

    //任务装配回调 匿名函数
    MissionCreator creator = [wm = out_wm.get(), sl = out_skills.get()](MissionDetails mission_details)->std::unique_ptr<Mission>
    {
        return Assembler::CreateMission(wm, sl, mission_details);
    };


    

    //仲裁层
    out_arb = std::make_unique<ArbitrateLayer>(out_wm.get(), creator);

}