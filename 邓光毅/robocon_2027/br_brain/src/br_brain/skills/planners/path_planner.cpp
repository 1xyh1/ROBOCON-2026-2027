#include "br_brain/skills/planners/path_planner.h"

#include <cmath>
#include <cstdlib>
#include <queue>
#include <algorithm>
#include <fstream>


PathPlanner::PathPlanner(WorldModel* world_model) : world_model(world_model) {}





//工具函数
bool PathPlanner::WorldToCell(double x, double y, int& col, int& row)
{
    col = static_cast<int>(std::floor((x - this->world_model->map_origin_x) / this->world_model->map_resolution));
    row = static_cast<int>(std::floor((y - this->world_model->map_origin_y) / this->world_model->map_resolution));

    return col >= 0 && col < this->world_model->map_width && row >= 0 && row < this->world_model->map_height;

}


int PathPlanner::ManhattanDistance(int n_idx, int t_idx, int width)
{
    //target_x/y 均是地图格子的行列索引 
    //表示二维位置的一维数组求行列索引  
    int c = n_idx % width;
    int r = n_idx / width;
    int target_x = t_idx % width;
    int target_y = t_idx / width;
    
   //返回格子单位下的曼哈顿距离
    return std::abs(c-target_x) + std::abs(r - target_y);

}


bool PathPlanner::HasLineOfSight(double x0, double y0, double x1, double y1)
{
    const double dx = x1 - x0;
    const double dy = y1 - y0;
    const double dist = std::sqrt(dx * dx + dy * dy);


    //步长取半个格子大小 避免漏格子
    const int steps = static_cast<int>(std::ceil(dist / (this->world_model->map_resolution * 0.5) ) );

    //特判 一般不可能steps为零  除非同一点
    if(steps == 0) return true;

    for(int i = 1; i <= steps; i++)
    {
        const double t = static_cast<double>(i) / steps;
        double nx = x0 + t * dx;
        double ny = y0 + t * dy;

        int col = 0;
        int row = 0;

        //如果其所在格子出界
        if(!this->WorldToCell(nx, ny, col, row)) return false;

        //如果其所在格子有障碍物
        if(this->inflated_cells[row * this->world_model->map_width + col] == WorldModel::CELL_OCCUPIED) return false;


    }

    return true;

}













//暴露接口的公有函数



bool PathPlanner::InflateMap()
{
    // if(!this->world_model->map_valid)
    // {
    //     std::cout<<"地图不存在 非法 将对空地图膨胀\n";
    //     //return false;
    // } 

    //若未加载地图 则按照默认空地图

    //使用路径规划器内部的 inflate_cells来存储膨胀后的地图
    this->inflated_cells = this->world_model->map_cells;

    //并入雷达更新的新障碍 
    for(size_t i = 0; i < this->inflated_cells.size(); i++)
    {
        if(this->world_model->dyn_cells[i] == 1)
        {
            this->inflated_cells[i] = WorldModel::CELL_OCCUPIED;
        }
    }

    

    //拷贝出来用于膨胀路径
    const std::vector<uint8_t> source = this->inflated_cells;

    //膨胀半径换算成格子单位    std::ceil函数向上取整
    //路径规划器内部的InfalteRadius是默认的 支持自己设定膨胀大小
    int r_cells = static_cast<int>(std::ceil(this->InflateRadius / this->world_model->map_resolution));

    for(int row = 0; row < this->world_model->map_height; row++)
    {
        for(int col = 0; col < this->world_model->map_width; col++)
        {
            if(source[row * this->world_model->map_width + col] != WorldModel::CELL_OCCUPIED)continue;

            for(int dr = -r_cells; dr <= r_cells; dr++)
            {
                for(int dc = -r_cells; dc <= r_cells; dc++)
                {
                    //圆形膨胀 方形遍历
                    if(dr * dr + dc * dc > r_cells * r_cells)continue;

                    int nr = row + dr;
                    int nc = col + dc;
                    if(nr < 0 || nr >= this->world_model->map_height || nc < 0 || nc >= this->world_model->map_width) continue;

                    this->inflated_cells[nr * this->world_model->map_width + nc] = WorldModel::CELL_OCCUPIED;
                }


            }

        }


    }


    return true;

}




//A*寻路算法
bool PathPlanner::PlanPath_AStar(double now_x, double now_y, double target_x, double target_y)
{
    this->out_path.clear();

    if(this->inflated_cells.empty())
    {
        std::cout<<"膨胀失败 无法寻路\n";
        return false;
    }

    
    //如果地图未成功加载或者膨胀不成功 返回false
    // if(!this->world_model->map_valid )
    // {
    //     std::cout<<"地图非法  将在默认空地图下寻路\n";
        
    // } 

    int n_col, n_row, t_col, t_row;

    //如果起点或者终点坐标对应的格子索引坐标越界 返回false
    if(!this->WorldToCell(now_x, now_y, n_col, n_row) || !this->WorldToCell(target_x, target_y, t_col, t_row)) return false;

    int w = this->world_model->map_width;
    int h = this->world_model->map_height;
    double res = this->world_model->map_resolution;
    auto& occ = this->inflated_cells;   //膨胀图的引用

    //格子索引 遍历也是同样
    int n_idx = n_row * w + n_col;
    int t_idx = t_row * w + t_col;

    //如果终点是不可达格子 返回false    允许起点(机器人当前所在点)因为雷达障碍更新 地图膨胀而被吞掉判定为CELL_OCCUPIED
    if(occ[t_idx] == WorldModel::CELL_OCCUPIED) return false;


    //创建结构体并重载大于号 用于优先队列
    struct Node
    {
        int index;
        int f;
        bool operator > (const Node& other) const
        {
            return f > other.f;
        }
    };

    std::priority_queue<Node, std::vector<Node>, std::greater<Node> > node;

    std::vector<int> walked(w * h, -1);                //表示走过的路的长度 用于计算代价
    std::vector<int> came_from(w * h, -1);             //表示从哪个格子走来 存储来源格子的索引

    
    constexpr int DX[4] = {1, 0, -1, 0};
    constexpr int DY[4] = {0, 1, 0, -1};

    //用期望走过的格子数代替代价直接省略代价系数 没有必要根据每格的分辨率来计算格子代价分数 
    
    bool Is_Acchieved = false;

    node.push({n_idx, 0 + this->ManhattanDistance(n_idx, t_idx, w)});
    walked[n_idx] = 0;
    while(!node.empty())
    {
        Node cur = node.top();
        node.pop();


        if(cur.index == t_idx)
        {
            Is_Acchieved = true;
            break;
        }

        int c = cur.index % w;
        int r = cur.index / w; 

        for(int i = 0; i < 4; i++)
        {
            int nc = c + DX[i];
            int nr = r + DY[i];
            int new_idx = nr * w + nc;
            
            //不符合的点不走
            if(nr < 0 || nr >= h || nc < 0 || nc >= w || occ[new_idx] == WorldModel::CELL_OCCUPIED)continue;

            //如果这个点走过了 另外一条路径再走时不会更优则不入队
            if(walked[new_idx] != -1 && walked[cur.index] + 1 >= walked[new_idx]) continue;

            walked[new_idx] = walked[cur.index] + 1;
            came_from[new_idx] = cur.index;

            node.push({new_idx, walked[new_idx] + this->ManhattanDistance(new_idx, t_idx, w)});
            

        }

    }




    if(!Is_Acchieved) return false;


    //根据came_from记录的寻路的依次格子路径 转为绝对坐标
    for(int idx = t_idx; idx != -1; idx = came_from[idx])
    {
        //地图原点在绝对坐标系下的坐标    0.5表示取格子中点位置
        double wx = this->world_model->map_origin_x + (idx % w + 0.5) * res;
        double wy = this->world_model->map_origin_y + (idx / w + 0.5) * res;
        this->out_path.push_back({wx, wy});
    }


    std::reverse(this->out_path.begin(), this->out_path.end());
    return true;


}





//路线剪枝函数
void PathPlanner::SimplifyPath()
{
    const  size_t n = this->out_path.size();

    

    
    //点数少于2 根本不用剪枝
    if(n < 2) return;

    //存放剪枝后的路线
    std::vector<std::pair<double, double>> simple;

    //当前剪枝的起点
    int anchor = 0;

    //先将起点push
    simple.push_back(this->out_path[anchor]);

    
    while(anchor < n - 1)
    {
        int best = -1;

        for(int idx = n - 1; idx > anchor; idx--)
        {
            double anchor_x = this->out_path[anchor].first;
            double anchor_y = this->out_path[anchor].second;

            double idx_x = this->out_path[idx].first;
            double idx_y = this->out_path[idx].second;

            //如果可以剪枝简化路线
            if(this->HasLineOfSight(anchor_x, anchor_y, idx_x, idx_y))
            {
                best = idx;
                break;
            }

        }

        //当机器人自身的点被膨胀污染或者其余因素判定为不可达时 会导致剪枝无法正常进行而死循环 故需要手动跳出剪枝
        //此时退化为未剪枝路径
        if(best < 0)best = anchor + 1;

        simple.push_back(this->out_path[best]);
        anchor = best;
        //std::cout<<"现在正在simplifypath\n";
    }

    //剪枝完将simple内的点赋值给out_path
    this->out_path.swap(simple);


}



bool PathPlanner::IsPathValid()
{
    const size_t n = this->out_path.size();

    if(n < 2) return true;      //无路径 路径合法 返回真

    for(size_t i = 0; i + 1 < n; i++)
    {
        if(!this->HasLineOfSight(this->out_path[i].first,   this->out_path[i].second,
                                 this->out_path[i + 1].first, this->out_path[i + 1].second))
        {
            return false;
        }
    }
    return true;



}



















//把膨胀图和规划路径画成pgm 路径涂灰205
void PathPlanner::DumpDebugPgm(const std::vector<std::pair<double, double> >& path, const std::string& file_path)
{
    if(this->inflated_cells.empty()) return;

    const int w = this->world_model->map_width;
    const int h = this->world_model->map_height;

    //复制膨胀图涂路径 不动膨胀图本体
    std::vector<uint8_t> img = this->inflated_cells;
    for(size_t i = 0; i < path.size(); i++)
    {
        int col = 0, row = 0;
        if(!this->WorldToCell(path[i].first, path[i].second, col, row)) continue;
        img[row * w + col] = 205;
    }

    //写文件时再翻回pgm行序 文件格式细节只准出现在这里
    std::ofstream f(file_path);
    f << "P2\n" << w << " " << h << "\n255\n";
    for(int pixel_row = h - 1; pixel_row >= 0; pixel_row--)
    {
        for(int col = 0; col < w; col++)
        {
            f << static_cast<int>(img[pixel_row * w + col]) << " ";
        }
        f << "\n";
    }
}



