#include <string>
#include <chrono>
#include <memory>
#include <cmath>


//ROS2的头文件
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>




//本项目头文件
#include "br_brain/common_defs.h"
#include "br_brain/arbitrate_layer.h"
#include "br_brain/skills/skill_layer.h"
#include "br_brain/assembler.h"



/*

循环中
仲裁层决断当前任务 任务基类执行


事件来源:订阅/event 话题std_msgs/String
节拍/频率 10ms 仲裁器更新频率


br_brain源码纯cpp不碰ros 仅有此文件连接ros2与决策


gazebo清理
killall -9 gzserver gzclient 2>/dev/null
ps aux | grep -E "gz[s]erver|gz[c]lient"    # 应该无输出
若有残留物
ls /dev/shm
rm /dev/shm/sem.fastrtps_* /dev/shm/fastrtps_* 2>/dev/null
ros2 daemon stop



ros2 topic pub /event std_msgs/msg/String "{data: start}" --once
ros2 topic pub /event std_msgs/msg/String "{data: buzzer}" --once

*/





class BrBrainNode : public rclcpp::Node
{
public:

    BrBrainNode() : Node("br_brain_node")
    {
        

        //加载场地地图 路径来自安装空间share目录 与当前工作目录无关
        const std::string map_yaml =
            ament_index_cpp::get_package_share_directory("br_brain") + "/maps/field_sim.yamlasd";
        

        Assembler::Assemble(this->world_model, this->skill_layer, this->arbitrate_layer, map_yaml);




        //裁判事件订阅 字符串映射为Event
        this->referee_sub = this->create_subscription<std_msgs::msg::String>(
            "/event", 10,
            std::bind(&BrBrainNode::RefereeCallback, this, std::placeholders::_1));

        


        this->odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&BrBrainNode::OdomCallback, this, std::placeholders::_1));

        this->amcl_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose", 10,
            std::bind(&BrBrainNode::AmclCallback, this, std::placeholders::_1));




        this->scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&BrBrainNode::ScanCallback, this, std::placeholders::_1)
        );






        this->cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10); 
        //发布话题


        //100hz 控制节拍
        this->timer = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&BrBrainNode::Tick, this));

        



        RCLCPP_INFO(this->get_logger(), "BR启动 100hz 事件话题 /event 输入 start/retry/buzzer");

    }





 




private:


    std::unique_ptr<WorldModel> world_model;
    std::unique_ptr<SkillLayer>skill_layer;
    std::unique_ptr<ArbitrateLayer> arbitrate_layer;



    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr referee_sub;     //监听外部事件
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;      //里程计监听
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;  //雷达扫描监听
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr amcl_sub;    //校正监听
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;    //速度控制
    rclcpp::TimerBase::SharedPtr timer;
    //std标准库中共享指针的别名 等价于 std::shared_ptr<Subscription<String>>


    Event current_event = Event::None;

    int print_tick = 0;

private:

    void RefereeCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        if(msg->data == "start")        this->current_event = Event::Start;
        else if(msg->data == "retry")   this->current_event = Event::RefereeRetry;
        else if(msg->data == "buzzer")  this->current_event = Event::Buzzer;
        //未识别的字符串静默忽略
    }



    //偏航角话题回调
    void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        //
        this->world_model->odom_x = msg->pose.pose.position.x;
        this->world_model->odom_y = msg->pose.pose.position.y;

        //朝向: 四元数转yaw 地面机器人只关心绕竖直轴的旋转
        const auto& q = msg->pose.pose.orientation;
        this->world_model->odom_yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                                 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        
    }

    //amcl锚点回调 需要地图与雷达数据
    void AmclCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        const auto& p = msg->pose.pose.position;
        const auto& q = msg->pose.pose.orientation;
        const double yaw = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
        this->world_model->UpdateCorrection(p.x, p.y, yaw);
    }
    




    //雷达扫描回调
    void ScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {

        this->world_model->scan_angle_min = msg->angle_min;
        this->world_model->scan_angle_increment = msg->angle_increment;


        for(size_t i = 0; i < msg->ranges.size() && i < WorldModel::SCAN_BEAM_COUNT; i++)
        {
            this->world_model->scan_ranges[i] = msg->ranges[i];
        }

        this->world_model->scan_stamp_s = rclcpp::Time(msg->header.stamp).seconds();
        this->world_model->UpdateDynamicLayer();

    }




    void Tick()
    {
        //取走并清空 单线程执行器下普通变量即可
        Event ev = this->current_event;
        this->current_event = Event::None;
        this->world_model->UpdatePosition();
        this->arbitrate_layer->ArbitrateUpdate(ev);

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x  = this->world_model->cmd_v;
        cmd.angular.z = this->world_model->cmd_w;
        this->cmd_vel_pub->publish(cmd);



        this->print_tick++;
        if(this->print_tick >= 100)
        {
            this->print_tick = 0;
            RCLCPP_INFO(this->get_logger(), "场地: x=%.3f y=%.3f yaw=%.3f | odom: %.3f %.3f %.3f | 修正边: %.3f %.3f %.3f",
                        this->world_model->pose_x,
                        this->world_model->pose_y,
                        this->world_model->pose_yaw,
                        this->world_model->odom_x,
                        this->world_model->odom_y,
                        this->world_model->odom_yaw,
                        this->world_model->corr_x,
                        this->world_model->corr_y,
                        this->world_model->corr_yaw);


            RCLCPP_INFO(this->get_logger(), "雷达 后%.2f 右%.2f 前%.2f 左%.2f",
                        this->world_model->scan_ranges[0],
                        this->world_model->scan_ranges[90],
                        this->world_model->scan_ranges[180],
                        this->world_model->scan_ranges[270]);

        }


    }








    

};






int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);                       //初始化ROS2底层
    rclcpp::spin(std::make_shared<BrBrainNode>());  //构造节点并进入执行器循环 阻塞至Ctrl+C
    rclcpp::shutdown();
    return 0;
}

