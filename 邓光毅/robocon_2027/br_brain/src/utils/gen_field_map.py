#!/usr/bin/env python3
#生成仿真场地地图 模拟导航组SLAM的交付物(pgm+yaml)
#几何与 br_sim/worlds/field_simplified.world 保持一致

import os

RES = 0.05            #分辨率: 一个格子0.05m
ORIGIN_X = -5.6       #地图左下角在场地坐标系的位置(围栏外沿)
ORIGIN_Y = -5.6
W = int(round(11.2 / RES))   #宽 224 格
H = int(round(11.2 / RES))   #高 224 格

FREE = 254            #白 = 可走 (ROS地图约定)
OCC  = 0              #黑 = 墙


#场地坐标 -> 格子下标  col向x增 row向y增(从下往上数)
#半开区间: 格子c覆盖[origin+c*res, origin+(c+1)*res) 边界值必须夹回
def world_to_cell(x, y):
    col = int(round((x - ORIGIN_X) / RES))
    row = int(round((y - ORIGIN_Y) / RES))
    col = max(0, min(W - 1, col))
    row = max(0, min(H - 1, row))
    return col, row



#把矩形(x1,y1)-(x2,y2)内所有格子填成val
def fill_rect(cells, x1, y1, x2, y2, val):
    c1, r1 = world_to_cell(x1, y1)
    c2, r2 = world_to_cell(x2, y2)
    for row in range(min(r1, r2), max(r1, r2) + 1):
        pixel_row = H - 1 - row    #关键: pgm第0行是图最上=y最大 行序翻转
        for col in range(min(c1, c2), max(c1, c2) + 1):
            cells[pixel_row][col] = val


def main():
    os.makedirs("maps", exist_ok=True)
    cells = [[FREE] * W for _ in range(H)]   #先整张图当可走

    #四面围栏 内壁±5.5 厚0.1
    fill_rect(cells, -5.6,  5.5,  5.6,  5.6, OCC)    #北
    fill_rect(cells, -5.6, -5.6,  5.6, -5.5, OCC)    #南
    fill_rect(cells,  5.5, -5.6,  5.6,  5.6, OCC)    #东
    fill_rect(cells, -5.6, -5.6, -5.5,  5.6, OCC)    #西

    #Z字内墙A 西端接西墙 东端到x=3.0 -> 东缺口x∈(3.0,5.5)
    fill_rect(cells, -5.5,  1.45, 3.0,  1.55, OCC)
    #Z字内墙B 西端x=-3.0 -> 西缺口x∈(-5.5,-3.0) 东端接东墙
    fill_rect(cells, -3.0, -1.55, 5.5, -1.45, OCC)

    #写pgm
    with open("maps/field_sim.pgm", "w") as f:
        f.write("P2\n")
        f.write("# field_sim 黑=墙 白=可走\n")
        f.write(f"{W} {H}\n")
        f.write("255\n")
        for row in cells:
            f.write(" ".join(str(v) for v in row) + "\n")

    #写yaml: 地图元数据 (nav2 map_server同款字段)
    with open("maps/field_sim.yaml", "w") as f:
        f.write("image: field_sim.pgm\n")
        f.write(f"resolution: {RES}\n")
        f.write(f"origin: [{ORIGIN_X}, {ORIGIN_Y}, 0.0]\n")
        f.write("negate: 0\n")
        f.write("occupied_thresh: 0.65\n")
        f.write("free_thresh: 0.25\n")

    #自检: 关键格子必须与场地几何一致
    checks = [
        ("场地中心(0,0)",          0.0,   0.0,  FREE),
        ("墙A中线(-1.25,1.5)",    -1.25,  1.5,  OCC),
        ("东缺口(4.0,1.5)",        4.0,   1.5,  FREE),
        ("西缺口(-4.0,-1.5)",     -4.0,  -1.5,  FREE),
        ("北墙(0,5.55)",           0.0,   5.55, OCC),
    ]
    for name, x, y, want in checks:
        col, row = world_to_cell(x, y)
        got = cells[H - 1 - row][col]
        mark = "OK  " if got == want else "FAIL"
        kind = "墙" if got == OCC else "可走"
        print(f"{mark} {name} -> {kind}")

    print(f"已生成 maps/field_sim.pgm + maps/field_sim.yaml ({W}x{H}格 {RES}m/格)")


if __name__ == "__main__":
    main()
