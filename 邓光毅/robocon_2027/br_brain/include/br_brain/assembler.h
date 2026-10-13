#pragma once

/*
全知类 引入所有头文件 禁用构造函数 定义静态方法

仅仅用于装配各个层 定义回调函数

其为全系统认识所有层(模块)的地方
只做装配接线传递指针 不含各种逻辑 永不实例化

*/




#include <memory>
#include <string>

//世界模型
#include "br_brain/world_model.h"
//仲裁层
#include "br_brain/arbitrate_layer.h"




//任务类
#include "br_brain/missions/mission_base.h"
#include "br_brain/missions/enter_field.h"
#include "br_brain/missions/build_earth.h"


//技能类
#include "br_brain/skills/skill_layer.h"
#include "br_brain/skills/skill_base.h"
#include "br_brain/skills/skill_chassis.h"


//功能函数
#include "utils/map_loader.h"





class Assembler
{
public:

    //组装 按顺序为 世界模型 技能层 仲裁层
    static void Assemble(std::unique_ptr<WorldModel>& out_wm,
                         std::unique_ptr<SkillLayer>& out_skills,
                         std::unique_ptr<ArbitrateLayer>& out_arb,
                         const std::string& map_yaml_path);


    

    //任务装配回调 即MissionCreator
    static std::unique_ptr<Mission> CreateMission(WorldModel* world_model, SkillLayer* skill_layer, MissionID id);


private:

    Assembler() = delete;   //不实例化






};
