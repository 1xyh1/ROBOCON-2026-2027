#include "br_brain/skills/skill_chassis.h"

#include <cmath>




SkillChassis::SkillChassis(WorldModel* world_model) : Skill(world_model) 
{
    this->path_planner = std::make_unique<PathPlanner>(world_model);
    this->path_planner->InflateMap();

    this->current_point = 0;
    this->is_path_planned = false;
}




//导航到目标点
bool SkillChassis::NavigateTo(double target_x, double target_y)
{

    bool need_plan = false;

    //目标点变了就要重新规划
    if(this->is_path_planned && (target_x != this->last_target_x || target_y != this->last_target_y))
    {
        this->is_path_planned = false;
        need_plan = true;
    }


    //世界动态地图障碍改变也要更新 然后判断原规划的路线在新地图下是否合法 否则重新规划
    if(this->last_dyn_version != this->world_model->dyn_version)
    {
        this->last_dyn_version = this->world_model->dyn_version;
        this->path_planner->InflateMap();

        //注意 这边没有路径合法时 考虑当地图更新后有更优的路径

        if(this->is_path_planned)
        {
            if(!this->path_planner->IsPathValid())
            {
                std::cout<<"[底盘] 路径被动态障碍截断 重规划\n";
                this->is_path_planned = false;
                need_plan = true;
            }

        }
        else
        {
            need_plan = true;
        }


    }



    if(need_plan && !this->is_path_planned)
    {
        double nx = this->world_model->pose_x;
        double ny = this->world_model->pose_y;

        if(this->path_planner->PlanPath_AStar(nx, ny, target_x, target_y))
        {
            this->path_planner->SimplifyPath();
            this->is_path_planned = true;
            this->current_point = 0;

            this->last_target_x = target_x;
            this->last_target_y = target_y;

        }
        else
        {
            //其当前没有可行路径
            std::cout<<"[底盘] 暂无可行路径 停下等待\n";
            //this->path_planner->DumpDebugPgm(this->path_planner->out_path, "/home/dgydgy/draft/dyn.pgm");
        }
    }

    if(!this->is_path_planned)
    {
        this->Stop();
        return false;
    }

    //当前需要去往的点
    double now_tx = this->path_planner->out_path[this->current_point].first;
    double now_ty = this->path_planner->out_path[this->current_point].second;

    if(this->GoToPoint(now_tx, now_ty))
    {
        this->current_point++;
        if(this->current_point == this->path_planner->out_path.size())
        {
            return true;
        }
    }
    
    

    return false;

}



bool SkillChassis::GoToPoint(double target_x, double target_y)
{
    //命令 到达某一点 暂时先随便写
    


    //极坐标系下 仅V 与 偏航角
    const double dx = target_x - this->world_model->pose_x;
    const double dy = target_y - this->world_model->pose_y;
    const double dist = std::sqrt(dx * dx + dy * dy);

    const double ARRIVE_DIST = 0.10;        //m 到达阈值 仿真里程计精度以内
    if(dist < ARRIVE_DIST)
    {
        this->Stop();
        return true;
    }

    //期望朝向 误差归一化到[-π, π]
    const double target_yaw = std::atan2(dy, dx);
    double yaw_err = target_yaw - this->world_model->pose_yaw;
    while(yaw_err >  M_PI) yaw_err -= 2.0 * M_PI;
    while(yaw_err < -M_PI) yaw_err += 2.0 * M_PI;



    //P比例控制 误差越大输出越大
    const double KP_V = 1.5;        //  m/s 每1m距离
    const double KP_W = 2.0;        //  rad/s 每1rad朝向误差
    const double V_MAX = 0.3;      //  m/s 仿真测试限速
    const double W_MAX = 1.2;       //  rad/s 角速度限速

    double v = KP_V * dist;
    double w = KP_W * yaw_err;


    if(v >  V_MAX) v =  V_MAX;      //限幅
    if(w >  W_MAX) w =  W_MAX;
    if(w < -W_MAX) w = -W_MAX;

    //朝向偏差太大时先原地转向 避免画大弧线冲过去
    if(std::fabs(yaw_err) > 0.4) v = 0.0;




    //先判断要不要刹停
    //刹停的时候只禁止前进 允许旋转
    
    if(this->Reflex_StopForwardVel())
    {
        v = 0;
    }







    this->world_model->cmd_v = v;
    this->world_model->cmd_w = w;

    return false;


}





void SkillChassis::Stop()
{
    this->world_model->cmd_v = 0;
    this->world_model->cmd_w = 0;

}


//反射层急刹函数实现 
bool SkillChassis::Reflex_StopForwardVel()
{
    static constexpr double REFLEX_FRONT_ANGLE = 0;      //正前方对应的波束角度 以实测校准为准
    static constexpr double REFLEX_HALF_ANGLE  = M_PI / 3.0; //扇区半宽 ±60°
    static constexpr double REFLEX_STOP_DIST   = 0.25;       //m 刹停阈值(雷达到障碍)

    //若无雷达数据 则按照雷达默认值无穷大判断

    const double d = this->world_model->MinRangeInSector(REFLEX_FRONT_ANGLE, REFLEX_HALF_ANGLE);
    if(d < REFLEX_STOP_DIST)
    {

        if(!this->reflex_stopped)
        {
            std::cout << "[技能:底盘] 反射刹停 最近障碍 " << d << " m\n";
            this->reflex_stopped = true;
        }

        return true;
        
    }

    if(this->reflex_stopped)
    {
        std::cout << "[技能:底盘] 刹停解除 恢复运动\n";
        this->reflex_stopped = false;
    }

    return false;


}


