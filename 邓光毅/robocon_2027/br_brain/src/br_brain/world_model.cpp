#include "br_brain/world_model.h"


#include <cmath>
#include <limits>   //limits函数


WorldModel::WorldModel()
{
    //各个模块默认值 保证模块间不用频繁相互询问存在与否

    //odom_x/y/yaw      0,0,0
    this->odom_x = 0.0;
    this->odom_y = 0.0;
    this->odom_yaw = 0.0;

    //corr_x/y/yaw   无AMCL时默认为0 即无偏差 退化为pose=odom
    this->corr_x = 0.0;
    this->corr_y = 0.0;
    this->corr_yaw = 0.0;


    //pose_x/y/yaw, scan_angle_forward	退化为odom
    this->pose_x = 0.0;
    this->pose_y = 0.0;
    this->pose_yaw = 0.0;
    this->scan_angle_forward = 0.0;

    //scan_ranges/角度  默认无穷大 即未扫描到目标
    for(int i = 0; i < WorldModel::SCAN_BEAM_COUNT; i++)
    {
        this->scan_ranges[i] = std::numeric_limits<double>::infinity();
    }
    this->scan_angle_min = 0.0;
    this->scan_angle_increment = 1.0;



    //地图块
    this->map_width = 300;              
    this->map_height = 300;             
    this->map_resolution = 0.05;
    this->map_origin_x = -7.5;    
    this->map_origin_y = -7.5;    
    this->map_cells.assign(90000, WorldModel::CELL_FREE);

    //dyn_cells/stamp/version雷达动态标记图     默认全空
    this->dyn_cells.assign(90000, 0);
    this->dyn_stamp.assign(90000, 0);
    this->dyn_version = 0;
    this->scan_stamp_s = 0;


    //cmd_v/w
    this->cmd_v = 0 ;
    this->cmd_w = 0 ;


};






//坐标偏移修正
void WorldModel::UpdateCorrection(double amcl_x, double amcl_y, double amcl_yaw)
{
    //旋转偏差θ_c = θ_amcl − θ_odom     注:二者均是在绝对坐标系下的偏差角 相减得到相对偏差
    this->corr_yaw = amcl_yaw - this->odom_yaw;
    this->corr_yaw = std::remainder(this->corr_yaw, 2.0 * M_PI);    
    //余数函数std::remainder(x, y) 返回 x - n*y 其中n为最接近x/y的整数 用以将其归一化到[-180, 180]
    const double Cos = std::cos(this->corr_yaw);
    const double Sin = std::sin(this->corr_yaw);
    /*
        旋转矩阵为
        cos -sin
        sin  cos
    
    */
    
   //平移偏差   由  偏差 + 旋转回正的值 = 校正值 得到
    this->corr_x = amcl_x - (Cos * this->odom_x - Sin * this->odom_y);
    this->corr_y = amcl_y - (Sin * this->odom_x + Cos * this->odom_y);



}

//默认直接按照amcl更新 如果amcl节点不存在 则由默认值自动降级为odom值
void WorldModel::UpdatePosition()
{

    const double Cos = std::cos(this->corr_yaw);
    const double Sin = std::sin(this->corr_yaw);

    this->pose_x = (Cos * this->odom_x - Sin * this->odom_y) + this->corr_x;
    this->pose_y = (Sin * this->odom_x + Cos * this->odom_y) + this->corr_y;
    this->pose_yaw = std::remainder(odom_yaw + corr_yaw, 2.0 * M_PI); 

    this->scan_angle_forward = this->pose_yaw;


}




//扇区检测函数
double WorldModel::MinRangeInSector(double center_angle, double half_width)
{
    //初始为无穷大
    double min_dist = std::numeric_limits<double>::infinity();

    for(int i = 0; i < this->SCAN_BEAM_COUNT; i++)
    {

        const double r = this->scan_ranges[i];

        //inf无穷大即超出雷达检测范围 或者 坏点 都不是障碍
        if(!std::isfinite(r) || r <= 0) continue;

        //第i束波在传感器系下的绝对角度
        const double angle = this->scan_angle_min + i * this->scan_angle_increment;

        //与扇区中心的角度差 归一化到[-180, 180]
        const double diff = std::remainder(angle - center_angle, 2.0 * M_PI);
        //std::fabs()专门处理浮点数的绝对值
        if(std::fabs(diff) > half_width) continue;

        if(r < min_dist) min_dist = r;

    }

    return min_dist;

}





void WorldModel::UpdateDynamicLayer()
{

    if(this->dyn_cells.size() != this->map_cells.size()) return;

    bool changed = false;


    for(int i = 0; i < this->SCAN_BEAM_COUNT; i++)
    {
        const double dist = this->scan_ranges[i];

        //超出雷达探测范围或探测出坏点  
        if(!std::isfinite(dist) || dist <= 0)continue;

        const double angle = this->scan_angle_min + i * this->scan_angle_increment + this->scan_angle_forward;
        const double ex = this->pose_x + dist * std::cos(angle);
        const double ey = this->pose_y + dist * std::sin(angle);

        int col = static_cast<int>(std::floor((ex - this->map_origin_x) / this->map_resolution));
        int row = static_cast<int>(std::floor((ey - this->map_origin_y) / this->map_resolution));        

        //越界则continue
        if(col < 0 || col >= this->map_width || row < 0 || row >= this->map_height) continue;


        const int idx = row * this->map_width + col;

        //本身是不可走或未知也continue
        if(this->map_cells[idx] != WorldModel::CELL_FREE) continue;
        //如果某格周围本身是障碍 那么不需要继续标记了
        if(this->NearOccupied(idx)) continue;



        if(this->dyn_cells[idx] == 0)
        {   
            this->dyn_cells[idx] = 1;
            changed = true;
        }

        //更新记录的时间
        this->dyn_stamp[idx] = this->scan_stamp_s;


    }

    //过期标记回收
    for(size_t i = 0; i < this->dyn_stamp.size(); i++)
    {
        //只清理标记的格子 未标记的格子的存在时长本身是无限的 否则不论有没有障碍 其始终会触发changed=true导致不断刷新
        if(this->dyn_cells[i] == 1 && this->scan_stamp_s - dyn_stamp[i] >= WorldModel::DYN_TTL)
        {
            this->dyn_cells[i] = 0;
            changed = true;
        }
    }

    if(changed)
    {
        this->dyn_version++;
    }


}






bool WorldModel::NearOccupied(int idx)
{
    const int w = this->map_width;
    const int row = idx / w;
    const int col = idx % w;

    for(int dr = -1; dr <= 1; dr++)
    {
        for(int dc = -1; dc <= 1; dc++)
        {
            if(dr == 0 && dc == 0) continue;

            const int r = row + dr;
            const int c = col + dc;
            if(r < 0 || r >= this->map_height || c < 0 || c >= this->map_width) continue;

            if(this->map_cells[r * w + c] == CELL_OCCUPIED) return true;
        }
    }
    return false;
}






