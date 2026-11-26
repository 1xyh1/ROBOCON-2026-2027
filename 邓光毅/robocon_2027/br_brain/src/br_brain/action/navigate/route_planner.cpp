#include "br_brain/action/navigate/route_planner.h"







static const Transition* FindTransition(const WorldModel* wm, TransitionKind kind, TeamSide team_side)
{
    for(const auto& t : wm->transitions)
    {
        if(t.kind != kind) continue;

        if(t.team_side == team_side) return &t;
    }
    return nullptr;
}






bool RoutePlanner::Plan(WorldModel* world_model, Point begin, Point end)
{
    out_path.clear();

    double x1 = begin.first;
    double y1 = begin.second;

    double x2 = end.first;
    double y2 = end.second;
    int c1,r1,c2,r2;
    if(!world_model->PointAtCell(x1, y1, c1, r1)) return false;
    if(!world_model->PointAtCell(x2, y2, c2, r2)) return false;

    //获取起点和终点对应的下标idx
    int idx_begin = r1 * world_model->map_width + c1;
    int idx_end = r2 * world_model->map_width + c2;

    FloorType floor_begin = world_model->cells_info[idx_begin].floor;
    FloorType floor_end = world_model->cells_info[idx_end].floor;

    const Transition* ramp  = FindTransition(world_model, TransitionKind::Ramp,  world_model->team_side);
    const Transition* stair = FindTransition(world_model, TransitionKind::Stair, world_model->team_side);

    //如果同层
    if(floor_begin == floor_end)
    {
        out_path.push_back(RouteStep{end, -1});
        return true;
    }

    //上楼
    if(static_cast<int>(floor_begin) < static_cast<int>(floor_end))
    {

        if(floor_begin == FloorType::L0)
            out_path.push_back(RouteStep{{ramp->lower_x, ramp->lower_y}, ramp->id});

        if(floor_end == FloorType::L2)
            out_path.push_back(RouteStep{{stair->lower_x, stair->lower_y}, stair->id});

    }
    else    //下楼
    {

        if(floor_begin == FloorType::L2)
            out_path.push_back(RouteStep{{stair->upper_x, stair->upper_y}, stair->id});

        if(floor_end == FloorType::L0)
            out_path.push_back(RouteStep{{ramp->upper_x, ramp->upper_y}, ramp->id});


    }


    out_path.push_back(RouteStep{end, -1});
    return true;

}