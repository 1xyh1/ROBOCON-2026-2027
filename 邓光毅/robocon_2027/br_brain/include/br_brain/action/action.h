#pragma once

/*
Action动作类

当分配好任务 需要按照任务执行相应动作的时候调用它 

动作可能有NavigateAction    处理导航动作 其包含底盘技能等 后续内部可替换为状态机


一个任务基本上都是顺序进行动作
如 导航到交接区 拿取方块

因此可以放入ActionSequence中来顺序调用



动作有开始 执行 取消的行为



*/

#include <iostream>


#include "br_brain/world_model.h"
#include "br_brain/skills/skill_layer.h"




enum class ActionState : uint8_t
{
    IDLE,
    RUNNING,
    DONE,
    FAILED,
};



class Action
{
public:

    Action(WorldModel* world_model);
    virtual ~Action() = default;

    virtual void Start() = 0;

    void StopAction();

    virtual ActionState Execute() = 0;






protected:

    WorldModel* world_model;


    //动作当前的状态
    ActionState current_state;

    //当前动作的技能池
    //存放指针副本 随动作对象析构删除副本
    //基类指针 所有被push进来的派生类指针自动向上转型
    std::vector<Skill*> current_skillpool;

    //技能的获取
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


