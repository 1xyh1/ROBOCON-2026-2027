#pragma once

//标准库中头文件
#include <vector>
#include <memory>

//引入领域词汇
#include "br_brain/common_defs.h"



#include "br_brain/world_model.h"
#include "br_brain/skills/skill_base.h"
#include "br_brain/skills/skill_layer.h"

/*
任务基类 一个Mission表示一个宏观任务  如建地块/盖天块/翻天块/放圣物

任务需要和世界交互 执行任务也需要技能 因此包含两头文件


生命周期: Enter()一次 -> Execute()每周期-> Stop()一次

任务通过技能层来驱动底盘机构等 不直接触碰硬件

*/



class Mission
{

public:

    enum class MissionStatus : uint8_t {Pending, Running, Done, Failed};    //任务的执行状态


    Mission(WorldModel* world_model, SkillLayer* skill_layer);
    virtual ~Mission() = default;

    virtual MissionID GetMissionID() = 0;     //返回对应的运行任务状态
    //派发任务时调用 从技能树中组合出技能并且执行
    virtual void Enter() = 0;  
    virtual MissionStatus Execute() = 0;            //任务执行 在每个周期中调用并且返回下一次的状态
    virtual void Stop() = 0;                        //在任务执行状态做完或者结束的时候调用一次 将机器人姿态调回安全默认状态
    virtual void Arb_Interrupt() = 0;               //专门响应仲裁层的打断操作

protected:
    WorldModel* world_model;
    SkillLayer* skill_layer;    //技能层借用入口

    bool Is_Mission_Normally = true;                //维护任务状态



    //技能集合 在派生类Enter里组合 存放是指针副本 随任务对象析构删除副本
    //基类指针 所有被push进来的派生类指针自动向上转型
    std::vector<Skill*> current_skillpool;  //本任务借用的技能清单

    //基类定义 所有技能安全停止 直接由基类实现 派生类中Stop将先调用该函数
    void StopAllSkills();





    //模板函数 按类型取技能 没有则nullptr 模板必须定义在头文件
    //在当前任务借用的技能集合中获取对应指针
    template<typename T>
    T* GetCurrentSkill()
    {
        for(auto& skill : this->current_skillpool)
        {
            if(T* p = dynamic_cast<T*>(skill)) return p;

        }
        return nullptr;
    }






private:



};