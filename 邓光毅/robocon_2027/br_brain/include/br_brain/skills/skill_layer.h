#pragma once


#include <vector>
#include <memory>

#include "br_brain/world_model.h"
#include "br_brain/skills/skill_base.h"


/*
技能层 独立层 在世界模型之后初始化
唯一拥有所有技能对象所有权的的对象  构造即完成全部技能的初始化
对外只提供按类型的借用 借用者不拥有 不delete


技能层不拥有世界模型指针 只传递世界模型指针用于构造技能基类的各个派生类

*/


class SkillLayer
{
public:

    explicit SkillLayer(WorldModel* world_model);   //explicit禁止隐式转换 禁止隐式拷贝构造
    ~SkillLayer();


    //按类型借技能 没有则nullptr  返回指针副本 所有权仍然在技能层
    template<typename T>
    T* Borrow()
    {
        for(auto& skill : this->skills)
        {
            if(T* p = dynamic_cast<T*>(skill.get())) return p;
        }
        return nullptr;

    }



private:


    std::vector<std::unique_ptr<Skill> > skills;


};



