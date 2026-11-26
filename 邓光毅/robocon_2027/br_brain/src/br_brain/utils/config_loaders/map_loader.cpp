#include "br_brain/utils/config_loader.h"

#include "br_brain/world_model.h"

#include <yaml-cpp/yaml.h>          //yaml解析 取代手写find(':')的分割

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <vector>



//============================================================
//  内部工具  static = 仅本文件可见 不污染全局命名空间
//============================================================




//从P5头部逐字符读一个记号(token)
//为什么不用 >> ?  P5头部之后紧跟的是裸像素字节 其中会出现 0x20(空格) 0x0A 0x0D 这些和空白字符
//完全相同的字节  用 >> 读到像素区就会被当成分隔符吞掉  所以头部只能逐字符读
//函数末尾会把token后的那一个空白字符一并消费掉  这正是P5规范里"头部与数据之间恰好一个空白"
static std::string ReadToken(std::ifstream& f)
{
    std::string tok;
    char c;

    //---- 第一段: 跳过空白与#注释 拿到token的首字符 ----
    while(f.get(c))
    {
        if(c == '#')
        {
            std::string rest;
            std::getline(f, rest);          //吞掉 # 之后到行尾的所有内容
            continue;
        }
        if(std::isspace(static_cast<unsigned char>(c))) continue;   //转unsigned再传isspace 避免负值UB
        tok.push_back(c);
        break;
    }

    if(tok.empty()) return tok;             //文件提前结束

    //---- 第二段: 一直累积到空白 该空白被消费掉 不再放回流中 ----
    while(f.get(c))
    {
        if(std::isspace(static_cast<unsigned char>(c))) break;
        tok.push_back(c);
    }

    return tok;
}




//读P5格式的pgm    只支持P5(二进制)  且只支持maxval=255
//输出的raw仍是pgm行序: raw[y*w+x] 的 y=0 是图顶(北)  尚未翻成世界行序
static bool ReadPgmP5(const std::filesystem::path& path, int& w, int& h, std::vector<uint8_t>& raw)
{
    std::ifstream f(path, std::ios::binary);    //二进制模式 否则Windows下会做CRLF转换 破坏像素数据
    if(!f.is_open())
    {
        std::cout << "[地图] 打不开 " << path << "\n";
        return false;
    }

    //---- 1. 头部三个记号: P5 / 宽 / 高 ----
    const std::string magic = ReadToken(f);
    if(magic != "P5")
    {
        std::cout << "[地图] 只支持P5 实际是 " << magic << "\n";
        return false;
    }

    w = std::stoi(ReadToken(f));
    h = std::stoi(ReadToken(f));
    const int maxv = std::stoi(ReadToken(f));   //读完这一句 其后的单个空白已被消费 流正好停在像素区开头

    if(w <= 0 || h <= 0)
    {
        std::cout << "[地图] 宽高非法 " << w << "x" << h << "\n";
        return false;
    }
    if(maxv != 255)     //maxval<256时每像素1字节 但值域不同 归一化公式要改  本项目不接这类图
    {
        std::cout << "[地图] 只支持maxval=255 实际是 " << maxv << "\n";
        return false;
    }

    //---- 2. 像素区: 一次性读 w*h 个裸字节 ----
    raw.assign(static_cast<size_t>(w) * h, 0);
    f.read(reinterpret_cast<char*>(raw.data()), static_cast<std::streamsize>(raw.size()));

    if(f.gcount() != static_cast<std::streamsize>(raw.size()))
    {
        std::cout << "[地图] 像素数据不足 读到 " << f.gcount()
                  << " 需要 " << raw.size() << "\n";
        return false;
    }

    return true;
}




//============================================================
//  掩膜灰度约定
//============================================================
//
//  三张掩膜与地图必须同尺寸 加载时统一做 行翻转 与地图共用 origin/resolution
//
//  三张掩膜共同约定
//      0   = 墙 / 不可通行
//      255 = 未标注(直接沿用底图的未知像素)
//      254 = "其余"底色 含义随掩膜而变 见下
//
//  所有灰度值统一取自 WorldModel::CELL_*  本文件不再另立一套常量
//
//  mask1 结构
//      61  = 楼梯 STAIR
//      100 = 斜坡 RAMP
//      254 = 平地 FLAT
//
//  mask2 层
//      230 = L0
//      204 = L1
//      179 = L2
//      100 = 斜坡格子 不归属任何一层
//      254 = 未命名的独立条带(东侧 6 条) 不归属任何一层
//
//  mask3 区域
//      128 = 公共区域 PUBLIC
//      254 = 非公共区域  按场地南北划分: y>0 蓝方  y<0 红方
//             (地图左下角 (-5.6,-5.6) 右上角 (5.6,5.6)  原点即左下角)


//读一张掩膜 校验尺寸并翻成世界行序  原始值不做任何解释
//tag 只用于打印 让人一眼看出是哪张掩膜出错
static bool LoadMaskRaster(const std::string& mask_path, const WorldModel& wm,
                           const char* tag, std::vector<uint8_t>& out)
{
    if(wm.map_width <= 0 || wm.map_height <= 0)
    {
        std::cout << "[掩膜] " << tag << " 必须在 LoadMap 之后加载\n";
        return false;
    }

    int w = 0, h = 0;
    std::vector<uint8_t> raw;
    if(!ReadPgmP5(std::filesystem::path(mask_path), w, h, raw)) return false;

    //尺寸必须与地图一致  不一致说明两张图不是同一套栅格 继续用就是错的
    if(w != wm.map_width || h != wm.map_height)
    {
        std::cout << "[掩膜] " << tag << " 尺寸不匹配 地图 " << wm.map_width << "x" << wm.map_height
                  << "  掩膜 " << w << "x" << h << "\n";
        return false;
    }

    //与地图同一套约定 world_row = h-1-pixel_row  只翻行 不改值
    //整行交换比逐格搬移少一次乘法 也不需要考虑"先把数组填成什么默认值"
    out = raw;
    for(int r = 0; r < h / 2; r++)
    {
        std::swap_ranges(out.begin() + static_cast<size_t>(r) * w,
                         out.begin() + static_cast<size_t>(r + 1) * w,
                         out.begin() + static_cast<size_t>(h - 1 - r) * w);
    }

    //启动时打印值分布  一眼确认掩膜是否正常 比事后调试便宜得多
    std::map<int, size_t> hist;
    for(uint8_t v : out) hist[v]++;

    std::cout << "[掩膜] " << tag << " 已加载 " << w << "x" << h << "  值分布:";
    for(const auto& kv : hist)
    {
        std::cout << " " << kv.first << ":" << kv.second;
    }
    std::cout << "\n";

    return true;
}




//============================================================
//  对外函数
//============================================================




//读nav2格式的yaml + 对应pgm  把场地地图装进世界模型
//yaml里 image 只是文件名  真实路径相对 yaml 自身所在目录
bool LoadMap(const std::string& yaml_path, WorldModel& wm)
{
    //---- 1. 解析yaml ----
    YAML::Node cfg;
    try
    {
        cfg = YAML::LoadFile(yaml_path);        //文件不存在/语法错都会抛 异常文本自带行号
    }
    catch(const YAML::Exception& e)
    {
        std::cout << "[地图] yaml解析失败 " << yaml_path << "\n" << e.what() << "\n";
        return false;
    }

    //必需键: 这三个缺了就没有几何意义 直接失败 不做猜测
    if(!cfg["image"] || !cfg["resolution"] || !cfg["origin"])
    {
        std::cout << "[地图] yaml缺少 image / resolution / origin\n";
        return false;
    }

    const std::string image_name = cfg["image"].as<std::string>();
    const double resolution = cfg["resolution"].as<double>();

    //origin 是三元组 [x, y, yaw]  表示地图左下角在世界坐标系的位置
    const YAML::Node origin = cfg["origin"];
    if(!origin.IsSequence() || origin.size() < 2)
    {
        std::cout << "[地图] origin格式非法 应为 [x, y, yaw]\n";
        return false;
    }
    const double origin_x = origin[0].as<double>();
    const double origin_y = origin[1].as<double>();
    //origin[2] 是地图朝向角 本框架要求地图不旋转 故不读取

    //可选键: 缺了用nav2默认值兜底 保证只有必需键的yaml也能跑
    const double occ_thresh  = cfg["occupied_thresh"] ? cfg["occupied_thresh"].as<double>() : 0.65;
    const double free_thresh = cfg["free_thresh"]     ? cfg["free_thresh"].as<double>()     : 0.25;
    //negate: 0=越黑越占据(常规地图)  1=越白越占据(少见的反色图)
    const bool negate = cfg["negate"] ? (cfg["negate"].as<int>() != 0) : false;

    if(resolution <= 0.0)
    {
        std::cout << "[地图] resolution非法 " << resolution << "\n";
        return false;
    }

    //---- 2. 读pgm ----
    //拼接规则交给filesystem: 若image本身是绝对路径 / 会自动丢弃左边 正好是我们要的语义
    const std::filesystem::path pgm_path = std::filesystem::path(yaml_path).parent_path() / image_name;

    int w = 0, h = 0;
    std::vector<uint8_t> raw;
    if(!ReadPgmP5(pgm_path, w, h, raw)) return false;

    //---- 3. 灰度归一化 + 行序翻转 ----
    //pgm行0是图顶(北)  我们约定数组行0是origin_y(南)
    //翻转只在加载这一次性吸收  之后全系统统一用 世界row*w+col 直查 各处不再各自翻转
    std::vector<uint8_t> cells(raw.size());
    for(int pixel_row = 0; pixel_row < h; pixel_row++)
    {
        const int world_row = h - 1 - pixel_row;            //图顶 -> 世界最北行(h-1)

        for(int col = 0; col < w; col++)
        {
            const int v = raw[pixel_row * w + col];         //0=黑 255=白

            //占据概率: negate=0时越黑概率越高  negate=1时反过来
            const double prob = negate ? (v / 255.0) : ((255.0 - v) / 255.0);

            uint8_t cell;
            if(prob >= occ_thresh)        cell = WorldModel::CELL_OCCUPIED;
            else if(prob <= free_thresh)  cell = WorldModel::CELL_FREE;
            else                          cell = WorldModel::CELL_UNKNOWN;

            cells[world_row * w + col] = cell;              //写入时就已经是世界行序
        }
    }

    //---- 4. 写入世界模型 ----
    wm.map_width      = w;
    wm.map_height     = h;
    wm.map_resolution = resolution;
    wm.map_origin_x   = origin_x;
    wm.map_origin_y   = origin_y;
    wm.map_cells      = std::move(cells);                   //搬移 免去几万字节的拷贝

    //---- 4.1 用底图初始化 cells_info ----
    //底图只决定 cell_type  结构与区域先保持默认(FLAT / PUBLIC) 等对应掩膜再覆盖
    wm.cells_info.assign(wm.map_cells.size(), MapCell{});
    for(size_t i = 0; i < wm.map_cells.size(); i++)
    {
        switch(wm.map_cells[i])
        {
            case WorldModel::CELL_OCCUPIED: wm.cells_info[i].cell_type = CellType::WALL;    break;
            case WorldModel::CELL_FREE:     wm.cells_info[i].cell_type = CellType::FREE;    break;
            default:                        wm.cells_info[i].cell_type = CellType::UNKNOWN; break;
        }
    }

    wm.real_cells = wm.map_cells;                           //real_cells = 纯静态地图 供膨胀用
    wm.dyn_cells.assign(static_cast<size_t>(w) * h, 0);     //动态层清空
    wm.dyn_stamp.assign(static_cast<size_t>(w) * h, 0.0);

    std::cout << "[地图] 已加载 " << w << "x" << h << " " << resolution
              << "m/格 origin(" << origin_x << "," << origin_y << ")\n";
    return true;
}




//mask1: 结构掩膜   只覆盖 structure_type  其余格子保持 FLAT
//必须在 LoadMap 之后调用  因为要借用地图的宽高来校验
bool LoadStructureMask(const std::string& mask_path, WorldModel& wm)
{
    std::vector<uint8_t> mask;
    if(!LoadMaskRaster(mask_path, wm, "mask1-结构", mask)) return false;

    if(wm.cells_info.size() != mask.size())
    {
        std::cout << "[掩膜] mask1-结构 cells_info 尺寸异常 请先 LoadMap\n";
        return false;
    }

    size_t stair = 0, ramp = 0;

    for(size_t i = 0; i < mask.size(); i++)
    {
        switch(mask[i])
        {
            case WorldModel::CELL_STAIR:
                wm.cells_info[i].structure_type = StructureType::STAIR;
                stair++;
                break;

            case WorldModel::CELL_RAMP:
                wm.cells_info[i].structure_type = StructureType::RAMP;
                ramp++;
                break;

            default:
                //0墙 / 254平地 / 255未标注 都不改  保留默认 FLAT
                break;
        }
    }

    std::cout << "[掩膜] mask1-结构 写入: 楼梯 " << stair << " 格  斜坡 " << ramp << " 格\n";
    return true;
}




//mask2: 楼层掩膜   只覆盖 floor
//墙 斜坡 未命名条带 未标注 一律记 NONE  表示"不归属某一层" 由规划器当作过渡区处理
bool LoadFloorMask(const std::string& mask_path, WorldModel& wm)
{
    std::vector<uint8_t> mask;
    if(!LoadMaskRaster(mask_path, wm, "mask2-楼层", mask)) return false;

    if(wm.cells_info.size() != mask.size())
    {
        std::cout << "[掩膜] mask2-楼层 cells_info 尺寸异常 请先 LoadMap\n";
        return false;
    }

    size_t l0 = 0, l1 = 0, l2 = 0, none = 0;

    for(size_t i = 0; i < mask.size(); i++)
    {
        FloorType floor = FloorType::NONE;

        switch(mask[i])
        {
            case WorldModel::CELL_L0: floor = FloorType::L0; l0++; break;
            case WorldModel::CELL_L1: floor = FloorType::L1; l1++; break;
            case WorldModel::CELL_L2: floor = FloorType::L2; l2++; break;

            default:
                //0墙 / 100斜坡 / 254未命名条带 / 255未标注
                none++;
                break;
        }

        wm.cells_info[i].floor = floor;
    }

    std::cout << "[掩膜] mask2-楼层 写入: L0 " << l0 << " 格  L1 " << l1
              << " 格  L2 " << l2 << " 格  不归层 " << none << " 格\n";
    return true;
}




//mask3: 区域掩膜   只覆盖 region_type
//128 = 公共   254 = 非公共   0/255 保持默认
//非公共区按场地南北分: y>0 蓝方  y<0 红方  与当前队别(team_side)无关
bool LoadRegionMask(const std::string& mask_path, WorldModel& wm)
{
    std::vector<uint8_t> mask;
    if(!LoadMaskRaster(mask_path, wm, "mask3-区域", mask)) return false;

    if(wm.cells_info.size() != mask.size())
    {
        std::cout << "[掩膜] mask3-区域 cells_info 尺寸异常 请先 LoadMap\n";
        return false;
    }

    size_t pub = 0, red = 0, blue = 0;

    for(size_t i = 0; i < mask.size(); i++)
    {
        switch(mask[i])
        {
            case WorldModel::CELL_PUBLIC:
                wm.cells_info[i].region_type = TeamSide::PUBLIC;
                pub++;
                break;

            case WorldModel::CELL_NON_PUBLIC:
            {
                //用格子中心的世界 y 判南北 避免格子下边界正好落在 y=0 时的歧义
                const int row = static_cast<int>(i / static_cast<size_t>(wm.map_width));
                const double cell_y = wm.map_origin_y
                                    + (row + 0.5) * wm.map_resolution;

                if(cell_y > 0.0)
                {
                    wm.cells_info[i].region_type = TeamSide::BLUE;
                    blue++;
                }
                else
                {
                    wm.cells_info[i].region_type = TeamSide::RED;
                    red++;
                }
                break;
            }

            default:
                //0墙 / 255未标注 不改  保留默认 PUBLIC
                break;
        }
    }

    std::cout << "[掩膜] mask3-区域 写入: 公共 " << pub << " 格  红方 " << red
              << " 格  蓝方 " << blue << " 格\n";
    return true;
}
