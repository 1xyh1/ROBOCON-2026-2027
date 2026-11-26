#pragma once

#include <cstdint>


//建造点的建造状态
enum class BuildState : uint8_t
{
    None,               //无方块
    OneEarth,           //一个地块
    TwoEarth,           //两个地块
    EnemySky,       //敌方塔
    FriendSky,      //我方塔

    Done,           //针对于圣物 放置结束即建成
};



namespace COMDEF
{

    constexpr const char* GetBuildStateName(BuildState state)
        {
            switch(state)
            {
                case BuildState::None : return "None";
                case BuildState::OneEarth : return "OneEarth";
                case BuildState::TwoEarth : return "TwoEarth";
                case BuildState::FriendSky : return "FriendSky";
                case BuildState::EnemySky : return "EnemySky";
                case BuildState::Done : return "Done";

            }


            return "Unknown";
        }


}




