#pragma once


/*

动作序列

用以在任务中顺序执行已注册的动作



*/

#include <iostream>
#include <memory>
#include <vector>

#include "br_brain/action/action.h"

class ActionSequence
{
public:


    ActionSequence();
    ~ActionSequence() = default;


    void AddAction(std::unique_ptr<Action> action);
    ActionState Execute();
    void StopActions();








private:

    //存储各个动作智能指针的vector容器
    std::vector<std::unique_ptr<Action> > actions;

    //动作执行到第几个了
    size_t index = 0;
    size_t actions_size = 0;
    //每个动作开始与否
    bool started = false;

    //整个序列完成与否
    bool finished = false;
    bool failed = false;





};



