#pragma once


#include <iostream>
#include <string>
#include <map>


#include "br_brain/world_model.h"



/*
打分依据  对每个任务的打分只依赖于世界模型当前的情况 与其他任务打分相互独立


score = 动作立刻得分 + 特殊情况分 - 自定系数λ * 成本




*/






class MissionStrategy
{
public:

    MissionStrategy(WorldModel* world_model);

    MissionDetails ComputeScores_ChooseMission();

private:

    void ProposeEnterField(std::vector<Option>& pool);
    void ProposeFetch(std::vector<Option>& pool);
    void ProposeBuildEarth(std::vector<Option>& pool);
    void ProposeBuildSky(std::vector<Option>& pool);
    void ProposeFlipSky(std::vector<Option>& pool);
    void ProposeMustika(std::vector<Option>& pool);


    uint8_t HowManyObj(ItemType item);
    void MyTowers(int& exclusive, int& shared) const;



    double ImmediateScore(Area area);       //动作立即得分
    double FutureScore(Area area);
    double FetchEarthValue();
    double FetchSkyValue();
    double ConditionalScore(Area area);     //特殊条件得分 即解锁圣物条件
    double CostScore(Area area);     //特殊条件得分 即解锁圣物条件


private:

    static constexpr double LAMBDA       = 2.0;   // 走1m的机会成本
    static constexpr double FUTURE_TRUST = 0.8;     //对未来的信任度
    static constexpr double SKY_SHARED_RISK = 0.6;   // 共享点天块风险折扣(专属点敌方永不可入=1.0)
    static constexpr double UNLOCK_BONUS = 300.0; // 圣物未定时 补上稀缺侧的塔的隐藏价值

    static constexpr double L1_ONE_EARTH = 10.0;
    static constexpr double L1_TWO_EARTH = 20.0;
    static constexpr double L1_SKY = 40.0;
    static constexpr double L2_ONE_EARTH = 20.0;
    static constexpr double L2_TWO_EARTH = 40.0;
    static constexpr double L2_SKY = 80.0;
    static constexpr double L2_MUSTIKA = 250;



private:

    WorldModel* world_model;





};







