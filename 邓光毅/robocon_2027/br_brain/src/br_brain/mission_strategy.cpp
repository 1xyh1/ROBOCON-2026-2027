#include "br_brain/mission_strategy.h"


#include <limits>
#include <vector>
#include <algorithm>
#include <cmath>

MissionStrategy::MissionStrategy(WorldModel* world_model) : world_model(world_model) {}






//判断身上是否有某物
uint8_t MissionStrategy::HowManyObj(ItemType item)
{
    uint8_t obj_num = 0;

    for(ItemType &obj : this->world_model->carrying_on)
    {
        if(obj == item) obj_num++;
    }
    
    return obj_num; 

}


void MissionStrategy::MyTowers(int& exclusive, int& shared) const
{
    exclusive = shared = 0;
    for(const auto& [area, info] : this->world_model->spot_info)
    {
        if (info.build_state != BuildState::FriendSky) continue;
        const bool is_excl = (area == Area::L1_North_ExclusiveBuildingArea || area == Area::L1_South_ExclusiveBuildingArea);
        is_excl ? exclusive++ : shared++;
    }
}






//立刻得分函数
double MissionStrategy::ImmediateScore(Area area)
{

    std::vector<Area> spotL1 = 
    {
        Area::L1_North_ExclusiveBuildingArea,
        Area::L1_South_ExclusiveBuildingArea,
        Area::L1_North_SharedBuildingArea,
        Area::L1_South_SharedBuildingArea,
        
    };

    std::vector<Area> spotL2 = 
    {
        Area::L2_Northeast_SharedBuildingArea,
        Area::L2_Northwest_SharedBuildingArea,
        Area::L2_Southeast_SharedBuildingArea,
        Area::L2_Southwest_SharedBuildingArea,
        Area::MustikaArea,

    };


    
    for(auto& k : spotL1)
    {
        if(k != area) continue;

        switch(this->world_model->spot_info.at(k).build_state)
        {
            case BuildState::None       : return MissionStrategy::L1_ONE_EARTH;
            case BuildState::OneEarth   : return MissionStrategy::L1_TWO_EARTH;      
        }

    }

    for(auto& k : spotL2)
    {
        if(k != area) continue;

        if(k == Area::MustikaArea)
        {
            return MissionStrategy::L2_MUSTIKA;
        }
        switch(this->world_model->spot_info.at(k).build_state)
        {
            case BuildState::None       : return MissionStrategy::L2_ONE_EARTH;
            case BuildState::OneEarth   : return MissionStrategy::L2_TWO_EARTH;      

        }


    }

    return 0.0;

}


//可能的未来得分
double MissionStrategy::FutureScore(Area area)
{
    const bool is_L2 = (area == Area::L2_Northeast_SharedBuildingArea ||
                        area == Area::L2_Northwest_SharedBuildingArea ||
                        area == Area::L2_Southeast_SharedBuildingArea ||
                        area == Area::L2_Southwest_SharedBuildingArea   );

    if(this->world_model->spot_info.at(area).build_state == BuildState::TwoEarth)
    {
        return (is_L2 ? MissionStrategy::L2_SKY : MissionStrategy::L1_SKY) * (COMDEF::BelongOf(area) == Belonging::Exclusive ? 1.0 : MissionStrategy::SKY_SHARED_RISK);
    }


    if(this->world_model->spot_info.at(area).build_state == BuildState::EnemySky)
    {
        return 2 * (is_L2 ? MissionStrategy::L2_SKY : MissionStrategy::L1_SKY) * (COMDEF::BelongOf(area) == Belonging::Exclusive ? 1.0 : MissionStrategy::SKY_SHARED_RISK);
    }





    return 0.0;

}


//拿取方块时的预期得分
// 取地块的未来分 = 最大的放置地块的得分 * 信任系数
double MissionStrategy::FetchEarthValue()
{
    //已经满载则退出 
    if (this->world_model->carrying_on.size() >= WorldModel::MAX_CARRYON) return 0.0;
    //可拿取的方块数
    int free_slots = WorldModel::MAX_CARRYON - (int)this->world_model->carrying_on.size();
    if (free_slots <= 0) return 0.0;
    bool has_sky = this->HowManyObj(ItemType::Sky) != 0;

    double v1 = 0.0, v2 = 0.0;                 // 最大的两个放置得分
    for (const auto& [area, info] : this->world_model->spot_info)
    {
        if (area == Area::MustikaArea) continue;
        // [地,地] 可放在None/OneEarth上 [地,天] 可放在OneEarth上并且封顶
        bool edible = has_sky ? (info.build_state == BuildState::OneEarth)
                                : (info.build_state == BuildState::None || info.build_state == BuildState::OneEarth);
        if (!edible) continue;
        double bite = this->ImmediateScore(area) + this->ConditionalScore(area);
        if (bite > v1)      { v2 = v1; v1 = bite; }
        else if (bite > v2) { v2 = bite; }
    }
    return MissionStrategy::FUTURE_TRUST * (free_slots >= 2 ? v1 + v2 : v1);
}

// 取天块的未来分 = 最好的封顶期望分 * 信任系数
double MissionStrategy::FetchSkyValue()
{
    if (this->HowManyObj(ItemType::Sky) != 0) return 0.0;               // 天块任意时刻最多1
    if (this->world_model->carrying_on.size() >= WorldModel::MAX_CARRYON) return 0.0;
    double best = 0.0;
    for (const auto& [area, info] : this->world_model->spot_info)
    {
        if (info.build_state != BuildState::TwoEarth) continue;         // 只封TWO_EARTH的顶
        best = std::max(best, this->FutureScore(area) + this->ConditionalScore(area));
    }
    return MissionStrategy::FUTURE_TRUST * best;
}







//用以判断是否需要优先完成获取圣物的条件的得分
double MissionStrategy::ConditionalScore(Area area)
{
    //以下if判断均为圣物已经被获取或放置的情况

    //已经达成获取圣物条件时return
    if(this->world_model->F_mustika_condition) return 0.0;

    //交接区已经送来了圣物
    if(this->world_model->Can_getmustika) return 0.0;

    //身上已有圣物时return
    if(this->HowManyObj(ItemType::Mustika) != 0) return 0.0;

    //圣物已经建成时return
    if(this->world_model->spot_info.at(Area::MustikaArea).build_state == BuildState::Done) return 0.0;


    int excl = 0;
    int shared = 0;
    this->MyTowers(excl, shared);
    
    if(excl < 1)
    {
        if(area == Area::L1_North_ExclusiveBuildingArea || area == Area::L1_South_ExclusiveBuildingArea)
        {
            return MissionStrategy::UNLOCK_BONUS;
        }
    }
    if(shared < 1)
    {
        if( area == Area::L1_North_SharedBuildingArea       ||
            area == Area::L1_South_SharedBuildingArea       ||
            area == Area::L2_Northeast_SharedBuildingArea   ||
            area == Area::L2_Northwest_SharedBuildingArea   ||
            area == Area::L2_Southeast_SharedBuildingArea   ||
            area == Area::L2_Southwest_SharedBuildingArea    )
        {
            return MissionStrategy::UNLOCK_BONUS;
        }
    }

    return 0.0;


}











//以下为各个任务的打分





MissionDetails MissionStrategy::ComputeScores_ChooseMission()
{

/*
任务清单:
    回交接区拿地块
    回交接区拿天块
    建造地块
    建造天块
    翻转天块
    搬运圣物


*/
    std::vector<Option> pool;   //任务池

    this->ProposeEnterField(pool);
    this->ProposeFetch(pool);
    this->ProposeBuildEarth(pool);
    this->ProposeBuildSky(pool);
    this->ProposeFlipSky(pool);
    this->ProposeMustika(pool);

    Option best;
    best.score = std::numeric_limits<double>::lowest();

    for(auto& opt : pool)
    {
        std::cout<<"[决策] 候选行: "<<COMDEF::GetID(opt.id)
                 <<" 目标区 "<<COMDEF::GetAreaName(opt.target_area)   //还没有Area名字表 先打编号
                 <<" 得分 "  <<opt.score<<"\n";


        if(opt.score > best.score)
        {
            best = opt;
        }
    }

    MissionDetails details;
    details.mission_id = MissionID::Idle;
    details.target_area = Area::None;
    details.target_x = this->world_model->pose_x;
    details.target_y = this->world_model->pose_y;


    //如果没有候选任务
    if(best.score == std::numeric_limits<double>::lowest())
    {
        return details;
    }


    details.mission_id = best.id;
    details.target_area = best.target_area;
    std::cout<<"[决策] 此时选择的任务 "<< COMDEF::GetID(details.mission_id)<<"\n";

    details.target_x = this->world_model->area_pose.at(best.target_area).first;
    details.target_y = this->world_model->area_pose.at(best.target_area).second;

    return details;


}








//大类任务的分配函数
void MissionStrategy::ProposeEnterField(std::vector<Option>& pool)
{
    if(this->world_model->field_entered == false)
    {
        Option o;
        o.id = MissionID::EnterField;
        o.target_area = Area::TransferArea;
        o.score = std::numeric_limits<double>::infinity();
        pool.push_back(o);
    }
}






void MissionStrategy::ProposeFetch(std::vector<Option>& pool)
{

    //工厂暂时没有Fetch类任务 整体停推 避免选中后无法分配
    // return;

    
    //暂定一个一个物块的取
    //当前已经达到最大携带数 就return
    uint8_t earth_num = this->HowManyObj(ItemType::Earth);
    uint8_t sky_num = this->HowManyObj(ItemType::Sky);
    
    if(earth_num + sky_num >= WorldModel::MAX_CARRYON) return;
    if(sky_num >= 1) return;
    //判断是否还能拿东西 如果能拿就一并拿了
    double AddScore = 0;

    //查看己方专属建造点位是否可以搭建天块 若有 则提高拿天块的得分
    std::vector<Area> spot = 
    {
        Area::L1_North_ExclusiveBuildingArea,
        Area::L1_South_ExclusiveBuildingArea,
        Area::L1_North_SharedBuildingArea,
        Area::L1_South_SharedBuildingArea,
        Area::L2_Northeast_SharedBuildingArea,
        Area::L2_Northwest_SharedBuildingArea,
        Area::L2_Southeast_SharedBuildingArea,
        Area::L2_Southwest_SharedBuildingArea,
    };
    bool can_sky = false;
    bool can_earth = false;
    for(auto& k : spot)
    {
        BuildState state = this->world_model->spot_info.at(k).build_state;
        if(state == BuildState::TwoEarth) can_sky = true;
        if(state == BuildState::None || state == BuildState::OneEarth ) can_earth = true;
    }

    Option o;
    
    //打分拿天块
    if(can_sky)
    {
        //时间紧张程度
        double tensity = 180.0 / this->world_model->race_time_left;

        o.id = MissionID::FetchSky;
        o.target_area = Area::TransferArea;
        o.score = this->FetchSkyValue();
    
        pool.push_back(o);

    }

    //打分拿地块
    if(can_earth)
    {
        //Option o;
        o.id = MissionID::FetchEarth;
        o.target_area = Area::TransferArea;
        o.score = this->FetchEarthValue();
        
        pool.push_back(o);

    }


    //打分拿圣物
    //未达成放置圣物条件    不由任务决策看 只看有没有自己有没有圣物
    // if(!this->world_model->F_mustika_condition) return;

    //交接区还没等到圣物
    if(!this->world_model->Can_getmustika) return;
    //身上已有圣物
    
    if(this->HowManyObj(ItemType::Mustika) != 0)return;
    //圣物已经放置完毕
    if(this->world_model->spot_info.at(Area::MustikaArea).build_state == BuildState::Done) return;

    //Option o;
    o.id = MissionID::FetchMustika;
    o.target_area = Area::TransferArea;
    o.score = 2000;

    pool.push_back(o);

}
















void MissionStrategy::ProposeBuildEarth(std::vector<Option>& pool)
{


    //如果身上没有地块  return
    if(this->HowManyObj(ItemType::Earth) == 0)return;






    std::vector<Area> spot = 
    {
        Area::L1_North_ExclusiveBuildingArea,
        Area::L1_South_ExclusiveBuildingArea,
        Area::L1_North_SharedBuildingArea,
        Area::L1_South_SharedBuildingArea,
        Area::L2_Northeast_SharedBuildingArea,
        Area::L2_Northwest_SharedBuildingArea,
        Area::L2_Southeast_SharedBuildingArea,
        Area::L2_Southwest_SharedBuildingArea,
    };

    for(auto& k : spot)
    {   
        BuildState state = this->world_model->spot_info.at(k).build_state;
        Belonging belong = COMDEF::BelongOf(k);
        double spotx = this->world_model->area_pose.at(k).first;
        double spoty = this->world_model->area_pose.at(k).second;

        double cost = std::fabs(this->world_model->pose_x - spotx) + std::fabs(this->world_model->pose_y - spoty);

        bool is_shared = (belong == Belonging::Shared);
        //如果塔不需要地块  continue
        if(state != BuildState::None && state != BuildState::OneEarth )  continue;
        

        Option o;
        o.id = MissionID::BuildEarth;
        o.target_area = k;
        o.score = this->ImmediateScore(k) + this->ConditionalScore(k);

        pool.push_back(o);


    }



}





void MissionStrategy::ProposeBuildSky(std::vector<Option>& pool)
{

    //如果身上没有天块  return
    if(this->HowManyObj(ItemType::Sky) == 0) return;



    //查看哪些建造点可以搭建天块
    std::vector<Area> spot = 
    {
        Area::L1_North_ExclusiveBuildingArea,
        Area::L1_South_ExclusiveBuildingArea,
        Area::L1_North_SharedBuildingArea,
        Area::L1_South_SharedBuildingArea,
        Area::L2_Northeast_SharedBuildingArea,
        Area::L2_Northwest_SharedBuildingArea,
        Area::L2_Southeast_SharedBuildingArea,
        Area::L2_Southwest_SharedBuildingArea,
    };


    for(auto& k : spot)
    {
        BuildState state = this->world_model->spot_info.at(k).build_state;

        //如果不可搭建天块
        if(state != BuildState::TwoEarth)continue;

        double spotx = this->world_model->area_pose.at(k).first;
        double spoty = this->world_model->area_pose.at(k).second;

        double cost = std::fabs(this->world_model->pose_x - spotx) + std::fabs(this->world_model->pose_y - spoty);

        //时间紧张程度
        double tensity = 180.0 / this->world_model->race_time_left;

        Option o;
        o.id = MissionID::BuildSky;
        o.target_area = k;
        o.score = this->FutureScore(k) + this->ConditionalScore(k);

        pool.push_back(o);

    }


}




void MissionStrategy::ProposeFlipSky(std::vector<Option>& pool)
{

    //如果有天块 无法翻转  continue
    if(this->HowManyObj(ItemType::Sky) != 0) return;


    std::vector<Area> spot = 
        {
            Area::L1_North_SharedBuildingArea,
            Area::L1_South_SharedBuildingArea,
            Area::L2_Northeast_SharedBuildingArea,
            Area::L2_Northwest_SharedBuildingArea,
            Area::L2_Southeast_SharedBuildingArea,
            Area::L2_Southwest_SharedBuildingArea,
        };


    for(auto& k : spot)
    {
        BuildState state = this->world_model->spot_info.at(k).build_state;

        //当敌方达成获取圣物条件时 优先翻转对方天块
        bool interrupt_mustika = this->world_model->E_mustika_condition 
        && this->world_model->spot_info.at(Area::MustikaArea).build_state != BuildState::Done;


        //只有是敌方的天块才需要翻转
        if(state != BuildState::EnemySky) continue;

        double tensity = 180.0 / this->world_model->race_time_left;

        Option o;
        o.id = MissionID::FlipSky;
        o.target_area = k;
        o.score = this->FutureScore(k) + this->ConditionalScore(k);
        pool.push_back(o);

    }


}


void MissionStrategy::ProposeMustika(std::vector<Option>& pool)
{
    //判断有没有达成放置圣物的条件
    if(!this->world_model->F_mustika_condition) return;

    //判断身上是否有圣物
    if(this->HowManyObj(ItemType::Mustika) == 0) return; 


    Option o;
    o.id = MissionID::Mustika;
    o.target_area = Area::MustikaArea;
    o.score = this->ImmediateScore(Area::MustikaArea);
    
    pool.push_back(o);


}










