


#include "br_brain/action/navigate/navigate_action.h"

#include "br_brain/skills/chassis/skill_chassis.h"


#include <cmath>



static const Transition* FindTransition(const WorldModel* wm, int id)
{
    for(const auto& t : wm->transitions)
    {
        if(t.id != id) continue;

        return &t;
    }
    return nullptr;
}









NavigateAction::NavigateAction(WorldModel* world_model, SkillLayer* skill_layer, NavigateRequire require)
: Action(world_model), require(require)
{

    this->current_skillpool.push_back(skill_layer->Borrow<SkillChassis>());
    

}



void NavigateAction::Start()
{

    std::cout<<"[NavigateAction] 开始动作 目标点 "<<"( "<<this->require.target_x<<" , "<<this->require.target_y<<" )\n";


    
    const Point begin{world_model->pose_x, world_model->pose_y};
    const Point end{require.target_x, require.target_y};
    
    if(!route_planner.Plan(world_model, begin, end))
    {
        std::cout << "[NavigateAction] 全局路线规划失败\n";
        current_state = ActionState::FAILED;
        return;
    }
    

    //打印日志
    std::cout << "[NavigateAction] 路线 " << route_planner.out_path.size() << " 步\n";
    for(size_t i = 0; i < route_planner.out_path.size(); i++)
    {
        const RouteStep& s = route_planner.out_path[i];
        std::cout << "  第 " << i << " 步 -> (" << s.target.first << ","
                  << s.target.second << ") transition=" << s.transition_id << "\n";
    }

    //清空状态机
    this->nav_fsm = FSM<NavState>{};
    //将规划出来的路径注册状态和转移

    //用来记录上一个状态机 判定是否是第一个状态
    NavState first_state;
    NavState prev_state;
    bool has_prev = false;

    using SR = FSM<NavState>::StateResult;
    size_t next_id = 0;
    SkillChassis* chassis = this->GetCurrentSkill<SkillChassis>();

    //判空 防止状态机异常
    if(!chassis)
    {
        std::cout << "[NavigateAction] 底盘技能指针不存在\n";
        current_state = ActionState::FAILED;
        return;
    }

    //RouteOut {target ; id}
    for(auto& step : this->route_planner.out_path)
    {
        //获取过渡区的信息 如果有
        const Transition* t = (step.transition_id != -1) ? FindTransition(this->world_model, step.transition_id) : nullptr;

        //防止出现需要进行transition的情况但是却没有找到对应的过渡口内容
        if(step.transition_id != -1 && !t)
        {
            std::cout << "[NavigateAction] 找不到 transition "
                    << step.transition_id << "\n";
            current_state = ActionState::FAILED;
            return;
        }

        //平地状态 前往目标点或者过渡区入口
        NavState flat;
        flat.id = next_id++;
        flat.mode = NavMode::FLAT;
        // flat.trans_mode = t ? t->kind : TransitionKind::None;
        flat.target_x = step.target.first;
        flat.target_y = step.target.second;

        FSM<NavState>::StateNode flat_node;

        flat_node.onEnter = [flat]()
        {
            std::cout << "[NavigateAction] 状态 " << flat.id
                      << " FLAT -> (" << flat.target_x << ","
                      << flat.target_y << ")\n";
        };

        flat_node.onUpdate = [chassis, flat]() {
            if(!chassis->NavigateTo(flat.target_x, flat.target_y))
                return SR::RUNNING;
            return SR::DONE;
        };
        this->nav_fsm.AddState(flat, flat_node);

        //如果是第一个注册的状态
        if(!has_prev)
        {
            first_state = flat;
            has_prev = true;
        }
        else    //如果已经有状态注册了 注册转移关系
        {
            this->nav_fsm.AddTransition(prev_state, SR::DONE, flat);

        }

        prev_state = flat;



        //上述的平地之间的导航 
        //以下为进入过渡点时的导航 根据是楼梯还是斜坡进行判定


        //如果需要进行transition动作    那就再注册一个transition状态
        if(t)
        {
            //step.target是入口 离 lower 近就是上楼 离 upper 近就是下楼

            //std::hypot用于计算几个数的平方和的平方根
            const double d_lower = std::hypot(step.target.first  - t->lower_x,
                                              step.target.second - t->lower_y);
            const double d_upper = std::hypot(step.target.first  - t->upper_x,
                                              step.target.second - t->upper_y);

            const bool going_up = (d_lower <= d_upper);

            //进入Transition状态后需要到达的退出点 即目标点
            const double exit_x = going_up ? t->upper_x : t->lower_x;
            const double exit_y = going_up ? t->upper_y : t->lower_y;


            //朝向 上楼朝upper 下楼朝lower
            const double axis_yaw = std::atan2(t->upper_y - t->lower_y,
                                               t->upper_x - t->lower_x);
            const double traverse_yaw = going_up
                ? axis_yaw
                : std::remainder(axis_yaw + M_PI, 2.0 * M_PI);


            NavState trans;
            trans.id = next_id++;
            trans.mode = NavMode::TRANSITION;
            // trans.trans_mode = t->kind;
            trans.target_x = exit_x;
            trans.target_y = exit_y;
            
            FSM<NavState>::StateNode trans_node;
            trans_node.onEnter = [trans]()
            {
                std::cout << "[NavigateAction] 状态 " << trans.id
                          << " TRANSITION -> (" << trans.target_x << ","
                          << trans.target_y << ")\n";
            };

            trans_node.onUpdate = [chassis, trans, traverse_yaw, t]() {

                //转向先转向 对准过渡区的方向 然后再进行动作

                if(!chassis->TurnTo(traverse_yaw))
                {
                    return SR::RUNNING;
                }

                if(t->kind == TransitionKind::Stair)
                {
                    
                    //待写具体的上楼梯及其判断成功与否的动作
                    std::cout<<"[NavFSM] 阶梯运动中\n";
                    return SR::RUNNING;
                }

                if(t->kind == TransitionKind::Ramp)
                {
                    
                    if(!chassis->NavigateTo(trans.target_x, trans.target_y))
                    {
                        std::cout<<"[NavFSM] 斜坡运动中\n";
                        return SR::RUNNING;
                    }

                }

                return SR::DONE;
            };

            this->nav_fsm.AddState(trans, trans_node);
            this->nav_fsm.AddTransition(flat, SR::DONE, trans);

            prev_state = trans;

        }


    }

    //没有任何状态注册 该情况只会在无法规划出路线时出现
    if(!has_prev)
    {
        std::cout << "[NavigateAction] 路线为空\n";
        current_state = ActionState::FAILED;
        return;
    }

    this->final_state_id = prev_state.id;



    if(!this->nav_fsm.Start(first_state))
    {
        std::cout << "[NavigateAction] 状态机启动失败\n";
        current_state = ActionState::FAILED;
        return;
    }





    this->current_state = ActionState::RUNNING;

}




ActionState NavigateAction::Execute()
{

    if(current_state == ActionState::FAILED) return ActionState::FAILED;
    if(current_state == ActionState::DONE) return ActionState::DONE;

    //状态机未启动 说明Start()时出现问题
    if(!this->nav_fsm.IsStarted())
    {
        std::cout << "[NavigateAction] 状态机未启动\n";
        current_state = ActionState::FAILED;
        return ActionState::FAILED;
    }

    const NavState before = this->nav_fsm.GetCurrentState();
    const FSM<NavState>::StateResult result = this->nav_fsm.Execute();

    if(result == FSM<NavState>::StateResult::FAILED)
    {
        std::cout << "[NavigateAction] 状态机失败\n";
        current_state = ActionState::FAILED;
        return ActionState::FAILED;
    }

    //只有当状态机的状态是最后一个状态同时该状态所有任务完成才算该导航动作完成
    if(result == FSM<NavState>::StateResult::DONE && before.id == this->final_state_id)
    {
        std::cout << "[NavigateAction] 全部路线完成\n";
        current_state = ActionState::DONE;
        return ActionState::DONE;
    }

    return ActionState::RUNNING;



    SkillChassis* chassis = this->GetCurrentSkill<SkillChassis>();

    if(!chassis)
    {
        std::cout<<"[NavigateAction]底盘技能指针不存在 \n";
        return ActionState::FAILED;
    }


    if(this->step_idx >= route_planner.out_path.size())
        return ActionState::DONE;

    //获取全局路线规划器的规划路径
    const RouteStep& step = route_planner.out_path[this->step_idx];

    //先走到当前步的终点
    if(!chassis->NavigateTo(step.target.first, step.target.second))
        return ActionState::RUNNING;    //还没到


    //运行至此处说明已经走到当前步终点

    //进行上下楼梯/斜坡动作
    if(step.transition_id != -1)
    {
        std::cout << "[NavigateAction] 到达 transition " << step.transition_id
                  << " 入口 特殊动作暂未实现 先放行\n";
        // TODO: 以后在这里调用 chassis->TraverseTransition(step.transition_id)

        const Transition* t = FindTransition(this->world_model, step.transition_id);

        







    }

    //运行完特殊的动作后

    this->step_idx++;

    if(this->step_idx >= route_planner.out_path.size())
    {
        std::cout << "[NavigateAction] 全部路线完成\n";
        return ActionState::DONE;
    }


    return ActionState::RUNNING;


}
