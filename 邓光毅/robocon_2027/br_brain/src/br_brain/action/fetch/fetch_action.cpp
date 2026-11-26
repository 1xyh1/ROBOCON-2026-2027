#include "br_brain/action/fetch/fetch_action.h"




/*

目前没有位置校准的模块
先写成一个简单的发送消息的模块

后续加入位置校准之后再改为状态机
后续也考虑增加握手协议 确保命令的发送是正常的






*/




FetchAction::FetchAction(WorldModel* world_model, SkillLayer* skill_layer, FetchRequire require)
: Action(world_model), require(require)
{

}



void FetchAction::Start()
{



}



ActionState FetchAction::Execute()
{


    return ActionState::DONE;
}
