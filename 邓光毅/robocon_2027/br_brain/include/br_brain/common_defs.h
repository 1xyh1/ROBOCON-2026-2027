#pragma once

#include <cstdint>

//全文件共享的领域词汇 

//任务状态
enum class MissionID : uint8_t
{
    Init,           //初始化 即BR从开始到自身场地
    Idle,           //闲置 即待机等待任务分配
    BuildEarth,     //放置地块
    CapSky,         //放置天块
    FlipSky,        //翻转天块
    PlaceOrb,       //放置圣物

    //以下为暂定任务
    EnterField,     //进入场地
    GoRetryField,   //返回重试区
};

//发生的事件
enum class Event : uint8_t
{
    None,           //默认无事件
    Start,          //开赛哨声
    RefereeRetry,   //裁判强制重试或者队伍要求重试
    Buzzer,         //终场蜂鸣

};

//拿着的物块 （待定）
enum class ItemType : uint8_t
{
    Earth,
    Sky,
    Orb,
};
