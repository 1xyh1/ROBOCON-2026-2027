#include "utils/map_loader.h"



#include "br_brain/world_model.h"


#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>



//static修饰的普通函数=只在本文件可见(内部链接) 避免污染全局命名空间


//跳过#注释 不断读出下一个空白分隔的记号 文件读完返回空串
static std::string NextToken(std::ifstream& f)
{
    std::string tok;
    while(f >> tok)
    {
        if(tok[0] == '#')            //注释: 丢掉#后到行尾的内容
        {
            std::string rest;
            std::getline(f, rest);
            continue;
        }
        return tok;
    }
    return "";
}






bool LoadMap(const std::string& yaml_path, WorldModel& wm)
{
    //---- 1. 解析yaml ----
    std::ifstream yml(yaml_path);
    if(!yml.is_open())
    {
        std::cout << "map_loader: 打不开 " << yaml_path << "\n";
        return false;
    }

    std::string image_name;
    double resolution = 0.0;
    double origin_x = 0.0, origin_y = 0.0;
    double occ_thresh = 0.65, free_thresh = 0.25;    //nav2默认值兜底

    std::string line;
    while(std::getline(yml, line))
    {
        const auto pos = line.find(':');
        if(pos == std::string::npos) continue;       //没有冒号的行跳过
        const std::string key = line.substr(0, pos);
        std::string val = line.substr(pos + 1);

        //去掉值两边的空白
        const auto first = val.find_first_not_of(" \t");
        const auto last  = val.find_last_not_of(" \t");
        if(first == std::string::npos) continue;
        val = val.substr(first, last - first + 1);

        if(key == "image")            image_name = val;
        else if(key == "resolution")  resolution = std::stod(val);
        else if(key == "origin")                                  //形如[-5.6, -5.6, 0.0]
        {
            val.erase(val.find('['), 1);
            val.erase(val.find(']'), 1);
            const auto c1 = val.find(',');
            const auto c2 = val.find(',', c1 + 1);
            origin_x = std::stod(val.substr(0, c1));
            origin_y = std::stod(val.substr(c1 + 1, c2 - c1 - 1));
            //第三分量是偏航角 地图用不到
        }
        else if(key == "occupied_thresh") occ_thresh  = std::stod(val);
        else if(key == "free_thresh")     free_thresh = std::stod(val);
    }

    if(image_name.empty() || resolution <= 0.0)
    {
        std::cout << "map_loader: yaml缺image或resolution\n";
        return false;
    }

    //pgm路径相对yaml所在目录
    const std::string dir = yaml_path.substr(0, yaml_path.find_last_of('/') + 1);
    const std::string pgm_path = dir + image_name;

    //---- 2. 读pgm头 (只支持P2=ASCII; P5二进制留到接真图时加分支) ----
    std::ifstream pgm(pgm_path);
    if(!pgm.is_open())
    {
        std::cout << "map_loader: 打不开 " << pgm_path << "\n";
        return false;
    }
    if(NextToken(pgm) != "P2")
    {
        std::cout << "map_loader: 只支持P2 实际是" << NextToken(pgm) << "\n";
        return false;
    }
    const int w = std::stoi(NextToken(pgm));
    const int h = std::stoi(NextToken(pgm));
    NextToken(pgm);                                  //maxval=255 用不到 跳过
    if(w <= 0 || h <= 0)
    {
        std::cout << "map_loader: 宽高非法\n";
        return false;
    }

    //---- 3. 逐格读入并归一化 ----
    //pgm的行0是图顶(北) 我们约定数组行0是origin_y(南)
    //翻转在加载这一次性吸收掉 之后全系统统一用 世界row*w+col 直查 各处不再各自翻转
    std::vector<uint8_t> cells(static_cast<size_t>(w) * h);
    for(int i = 0; i < w * h; i++)
    {
        const int v = std::stoi(NextToken(pgm));
        const double prob = (255.0 - v) / 255.0;     //negate=0约定: 越黑占据概率越高

        uint8_t cell;
        if(prob >= occ_thresh)       cell = WorldModel::CELL_OCCUPIED;
        else if(prob <= free_thresh) cell = WorldModel::CELL_FREE;
        else                         cell = WorldModel::CELL_UNKNOWN;

        const int col       = i % w;                 //pgm第i个像素的列
        const int pixel_row = i / w;                 //pgm行 0=图顶
        const int world_row = h - 1 - pixel_row;     //翻成世界行 0=图南
        cells[world_row * w + col] = cell;
    }


    //---- 4. 写入WorldModel ----
    wm.map_width = w;
    wm.map_height = h;
    wm.map_resolution = resolution;
    wm.map_origin_x = origin_x;
    wm.map_origin_y = origin_y;
    wm.map_cells = std::move(cells);                 //搬移: 免去几万字节的拷贝

    wm.dyn_cells.assign(static_cast<size_t>(w) * h, 0);
    wm.dyn_stamp.assign(static_cast<size_t>(w) * h, 0.0);

    std::cout << "map_loader: 已加载 " << w << "x" << h << " "
              << resolution << "m/格 origin(" << origin_x << "," << origin_y << ")\n";
    return true;
}





uint8_t CellAt(const WorldModel& wm, double x, double y)
{
    
    //点落在哪一格: 半开区间 用floor
    int col = static_cast<int>(std::floor((x - wm.map_origin_x) / wm.map_resolution));
    int row = static_cast<int>(std::floor((y - wm.map_origin_y) / wm.map_resolution));

    if(col < 0 || col >= wm.map_width || row < 0 || row >= wm.map_height)
        return WorldModel::CELL_UNKNOWN;

    return wm.map_cells[row * wm.map_width + col];   //存储已是世界行序 直接直查

}
