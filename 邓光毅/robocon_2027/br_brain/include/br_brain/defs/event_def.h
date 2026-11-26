#pragma once

#include <cstdint>

//发生的事件
enum class Event : uint8_t
{
    None,           //默认无事件
    Start,          //开赛哨声
    RefereeRetry,   //裁判强制重试或者队伍要求重试
    Buzzer,         //终场蜂鸣

};