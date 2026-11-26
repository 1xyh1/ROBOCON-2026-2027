#pragma once


/*
    有限状态机模板

    需要注册状态和转移 允许不传入onEnter和onExit方法 必须传入onUpdate

*/
#include <functional>
#include <map>
#include <cstdint>
#include <utility>





template<typename State>
class FSM
{
public:


    enum class StateResult : uint8_t
    {
       IDLE,
       RUNNING,
       DONE,
       FAILED,

    };



    using Behave = std::function<void()>;

    using Update = std::function<StateResult()>;

    

    struct StateNode
    {
        Behave onEnter;
        Update onUpdate;
        Behave onExit;

        std::map<StateResult, State> transitions;
    };

    bool AddState(State state, StateNode state_node)
    {
        //状态已经存在
        if (states.count(state)) return false;

        this->states[state] = state_node;
        return true;
    }



    bool AddTransition(State from, StateResult result, State to)
    {
        //如果转移前状态或转移后状态不存在 返回false
        if(!states.count(from) || !states.count(to)) return false;

        states.at(from).transitions[result] = to;

        return true;

    }
    bool Start(State initial)
    {
        auto it = states.find(initial);

        if(it == states.end()) return false;

        currentState = initial;
        started = true;

        if(it->second.onEnter)it->second.onEnter();

        return true;

    }
    StateResult Execute()
    {
        if(!this->started) return StateResult::IDLE;

        auto it = states.find(this->currentState);
        if(it == states.end() || !it->second.onUpdate) return StateResult::FAILED;

        const StateResult result = it->second.onUpdate(); 

        auto tr = it->second.transitions.find(result);
        if (tr == it->second.transitions.end()) return result;    //无迁移

        if (it->second.onExit) it->second.onExit();

        this->currentState = tr->second;
        auto next = states.find(this->currentState);
        if (next == states.end() || !next->second.onUpdate)
        {
            started = false;
            return StateResult::FAILED;    //配置错误: 迁移到了未注册/无更新函数的状态
        }
        if (next->second.onEnter) next->second.onEnter();

        return result;

    }


    void Stop()
    {
        if(!started) return;

        auto it = states.find(this->currentState);
        if(it != states.end() && it->second.onExit) it->second.onExit();
        started = false;
    }

    State GetCurrentState() const { return currentState; }
    bool IsStarted() const { return started; }

private:

    std::map<State, StateNode> states;


    State currentState{};
    bool started = false;


};

