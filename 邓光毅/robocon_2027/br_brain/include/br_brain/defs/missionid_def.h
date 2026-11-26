#pragma once

#include <cstdint>
#include "br_brain/defs/area_def.h"


//任务状态
enum class MissionID : uint8_t
{
    Init,               //初始化 即BR从开始到自身场地
    Idle,               //闲置 即待机等待任务分配
    BuildEarth,         //放置地块
    BuildSky,           //放置天块
    FlipSky,            //翻转天块
    Mustika,            //放置圣物
    
    //以下为暂定任务    
    EnterField,         //进入场地
    GoRetryField,       //返回重试区
    FetchEarth,         //回交接区拿地块
    FetchSky,           
    FetchMustika,



    Debug,
};


//任务层详细信息 用于仲裁层分配任务
struct MissionDetails
{
    MissionID mission_id = MissionID::Idle;

    Area target_area = Area::None;

    double target_x = 0.0;
    double target_y = 0.0;



};



struct Option
{
    MissionID id;           //任务大类
    Area target_area;
    double score;           //任务分数
};




namespace COMDEF
{

    constexpr const char* GetID(MissionID id)
        {
            switch(id)
            {
                case MissionID::Init: return "Init";
                case MissionID::Idle: return "Idle";
                case MissionID::BuildEarth: return "BuildEarth";
                case MissionID::BuildSky: return "BuildSky";
                case MissionID::FlipSky: return "FlipSky";
                case MissionID::Mustika: return "Mustika";
                case MissionID::EnterField: return "EnterField";
                case MissionID::GoRetryField: return "GoRetryField";
                case MissionID::FetchEarth: return "FetchEarth";
                case MissionID::FetchSky: return "FetchSky";
                case MissionID::FetchMustika: return "FetchMustika";
                
                
            }
    
            return "Unknown";
        }
}

