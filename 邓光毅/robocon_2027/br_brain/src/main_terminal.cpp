#include <iostream>
#include <atomic>
#include <chrono>
#include <string>
#include <thread>


//#include "world_model.h"
#include "br_brain/arbitrate_layer.h"
/* 
循环中
仲裁层决断当前任务 任务基类执行

当前事件来源
先用标准输入模拟裁判系统 终端输入 start / retry / buzzer 回车触发
后续接入ROS2后 这里换成订阅 /event 话题

*/

static std::atomic<Event> typing_event {Event::None};   //原子类型 多线程写入安全

static void EventReaderThread()
{
    std::string line;
    while(std::getline(std::cin, line))
    {
        std::cout << "[事件] 收到输入: '" << line << "'\n";   //临时调试
        if(line == "start") typing_event = Event::Start;
        if(line == "retry") typing_event = Event::RefereeRetry;
        if(line == "buzzer") typing_event = Event::Buzzer;

    }
}




int main()
{

    std::cout<<"BR启动 更新频率100hz 事件输入: start/retry/buzzer\n";



    WorldModel world_model;
    ArbitrateLayer arbitrate_layer(&world_model);

    std::thread reader(EventReaderThread);
    reader.detach();    //独立运行至程序终止 而非join堵塞

    const auto HzPeriod = std::chrono::milliseconds(10);    //每周期的时间
    auto next_tick = std::chrono::steady_clock::now();      //该周期到下一周期的时间戳

    
    while(true)
    {
        Event ev = typing_event.exchange(Event::None);  //exchange是原子操作 将值清空为Event::None并返回旧值 
        //注意 在一个周期的时间内如果有两个事件 会直接覆盖(暂时先不考虑高频连续事件出现 一般不可能)
        arbitrate_layer.ArbitrateUpdate(ev);

        next_tick += HzPeriod;
        std::this_thread::sleep_until(next_tick);   
        //动态刷新 使得帧率稳定

    }




}