# 建图与全局重定位：处理方法审查及交付记录

日期：2026-11-03。当前主机：`RobopioneerAMD`，SSH `10.151.103.62`。

**本轮已经把 CPU 全地图搜索、GICP 精配准、回环位姿图后端和地图保存接进现有启动链，并在小主机运行。真实录包已完成建图、保存、重新加载、自动首锚点和连续定位。FAST-LIVO2 的运动融合问题仍未修好。**

当前应用、安装和脚本位于 `/home/nvidia/DICodex/ROBOCON2027/algorithm-review-20261103-session1`；Windows 同名目录在 `D:\DICodex\ROBOCON2027`。稳定入口为该项目根目录的 `run_slam.sh`、`run_rviz.sh`、`run_runtime.sh`。11/01 安装保留为旧版本，当前入口加载新 overlay。

## 1. 修正的问题前提

“已有 GICP”不等于“能够在地图任意位置找回”。原 GICP 只在种子附近收敛；主控和资产解析还要求显式起始种子。历史锚点附近的小范围搜索不能覆盖搬动后的冷启动。

“有固定地图纠偏”也不等于“有建图消漂后端”。此前 FAST-LIO 维护的是局部里程计地图，没有接回环位姿图。固定地图 GICP 可以纠正当前 `map→odom`，不能回头改好一张已经因里程计误差变形的地图。

“点云分布三维”不等于“六自由度都可观”。本轮构造了墙面加地面的走廊：点分布协方差最小/最大特征值比为 0.08246，超过现有 0.003 门限，但理想表面法向信息矩阵秩为 5，沿走廊方向仍不受约束。这是数学反例，不是一次 PCL 运行结果。[反例与修复前后代价](../algorithm-review-20261103-session1/acceptance-counterexamples.json)

## 2. 实际使用的算法

| 环节 | 本轮实现 | 作用与依据 |
| --- | --- | --- |
| 无视觉里程计 | 既有 FAST-LIO2，MID-360 点云和内置 IMU | IMU 传播、逐点去畸变、迭代误差状态滤波、点到平面残差、ikd-tree 局部地图。[论文](https://arxiv.org/abs/2107.06829)、[源码](https://github.com/hku-mars/FAST_LIO) |
| 建图后端 | FASTLIO2_ROS2 的位姿图核心 + GTSAM iSAM2 | 关键帧与里程计因子、空间邻近回环提议、GICP 验证、Huber 回环因子、优化地图与轨迹。[上游核心](https://github.com/liangheming/FASTLIO2_ROS2/tree/7baa59f7339ba88ce3b646fe35b23f61031c5d3f/pgo)、[GTSAM](https://github.com/borglab/gtsam) |
| 无初值全局重定位 | 3D-BBS 的 CPU 分支 | 重力对齐后搜索全地图 XYZ 和完整航向；保留不同候选区域，再做精配准。[论文](https://arxiv.org/abs/2310.10023)、[固定源码版本](https://github.com/KOKIAOKI/3d_bbs/tree/41529a34a2fb9618b5ff560fb3c2363f1615666d) |
| 精配准与连续纠偏 | PCL Generalized ICP | 用局部协方差加权优化对应点；按整帧代价、重合率等评估，发布 `map→odom`。[原论文](https://www.roboticsproceedings.org/rss05/p21.pdf) |
| 视觉融合 | 既有隔离 FAST-LIVO2 | 雷达几何与图像光度残差更新共享状态；本轮没有把它替换为正式定位。[论文](https://arxiv.org/abs/2408.14035) |
| 显示 | ROS 2 RViz2 | 相对定位、固定地图定位、建图、融合四套视图。[官方源码](https://github.com/ros2/rviz/tree/humble) |

CPU 路线适配当前 AMD 主机，没有引入 CUDA。两份实际复用代码固定到上述提交，保留 MIT 许可证与原始下载；修改说明在 [third_party/README.md](../algorithm-review-20261103-session1/worktree/robot_ws/src/robocon_global_localization/third_party/README.md)。

## 3. 逐段处理方法检查

### 3.1 FAST-LIO：传播、去畸变和 Jacobian

实际状态顺序为位置、旋转、雷达到 IMU 的旋转/平移、速度、陀螺偏置、加计偏置、重力方向；不能套用 LIVO 的状态下标。

传播关系为 `p_dot=v`、`R_dot=R[gyro-bg]x`、`v_dot=R(acc-ba)+g`。点云按逐点时间相对扫描末端补偿运动；`CustomMsg.offset_time` 保持纳秒语义，经前处理转换使用。本轮没有用录包接收时间替代逐点测量时间。

点到平面残差写成 `r=nᵀ[R(R_IL p_L+t_IL)+p]+d`。令 `p_I=R_IL p_L+t_IL`、`C=Rᵀn`，对应位置、右扰动旋转、外参旋转、外参平移的 Jacobian 为 `[n, [p_I]x C, [p_L]x R_ILᵀ C, C]`。源代码与此一致；100 组中心差分检查的最大绝对误差为 `1.8725e-9`。[可运行检查与源哈希](../algorithm-review-20261103-session1/lio-jacobian-check.json)

初始重力估计不自动保证旧 odom 地图的 Z 轴朝上。因此新建图端先根据初始 IMU 与里程计姿态计算固定的重力对齐旋转，再把关键帧放入地图。地图明确保存 `frame: map`、`up_axis: z`；重新定位时按同一约定对齐重力。

当前 SLAM 入口使用已许可 B3 的两个 0.1 m 体素参数和关闭在线外参估计；原 FAST-LIO 参数文件与二进制保留。该选择不构成 1 cm 精度结论。

### 3.2 GICP：修复只看内点 RMSE 的选解偏差

原方法把未匹配点从 RMSE 中移除，再优先选 RMSE 最小的候选。反例中正确解覆盖 100%、RMSE 2 cm，错误重复结构仅覆盖 55%、RMSE 为 0；旧排序反而选择错误位置。

本轮候选排序使用整帧截断代价 `sqrt(sum(min(d_i², cutoff²))/N)`，未匹配点贡献现有 0.2 m 截断距离。反例中正确解代价为 0.020 m，错误解为 0.134 m，排序得到修正；公开质量仍报告原有内点 RMSE 和重合率，没有偷偷更换其含义。[实现](../algorithm-review-20261103-session1/worktree/robot_ws/src/robocon_global_localization/src/gicp_matcher.cpp)

旧连续匹配还存在自锁：匹配失败清掉 committed，下一轮却要求 committed 仍有效才运行。现在保留旧锚点作为搜索初值，只有新鲜匹配成功后才恢复发布。原生 ROS 检查覆盖坏帧超过有效期后恢复正常点云的过程。

### 3.3 全局搜索：真正去掉起点依赖

空 `Relocalize.seeds` 现在请求全地图搜索；主控在 SELF_CHECK 自动发送空种子。资产解析和 launch 已移除“必须先有人工起点”的依赖。直接 Action 仍接受手工种子用于局部精配准。[入口](../algorithm-review-20261103-session1/worktree/scripts/launch_slam.py)、[主控](../algorithm-review-20261103-session1/worktree/robot_ws/src/robocon_executive/robocon_executive/mission_executive_node.py)

搜索使用最近最多 50 个 IMU 比力样本确定朝上方向，随后分支定界搜索 XYZ 与完整航向。搜索结束后重新取当前扫描，并用搜索前后 LIO 的相对运动传播初值，再做 GICP；不把已经过期的搜索起始帧直接提交为当前定位。

上游只返回一个最优位姿，不足以检查重复场景。本轮保留分数接近的不同区域，交给已有 GICP 歧义比较；区域过多或两个不同位置同样匹配时不提交 TF。原生用例验证了相隔 10 m 的两个复制区域被识别为歧义。

此外修复了复用 CPU 源码的有符号体素哈希乘法溢出，计时改为单调时钟。搜索依赖地图的 Z-up 约定；重定位初始化时应停稳，明显线加速度会污染重力方向。它不是完全抛开 IMU 的六维暴力搜索。

### 3.4 建图：接入回环与保存可重建数据

新适配节点直接接收现有 `/cloud_registered_body` 和同时间戳 `/Odometry`，没有重新去畸变或重复应用雷达外参。按 0.5 m/10° 提取关键帧；回环候选仍按已有位置先验提出，GICP 检查后加入 iSAM2。室内适配端把时间排除从上游 60 s 调为 10 s，避免短场地必须等一分钟才可能闭环。[实现](../algorithm-review-20261103-session1/worktree/robot_ws/src/robocon_global_localization/src/map_builder_node.cpp)

保存内容包括 `map.pcd`、`map.yaml`、`fast_lio.yaml`、优化 `poses.csv` 和逐关键帧本体 PCD；`/mapping/save` 保存，正常退出也保存。输出地图重建使用优化位姿，避免把旧拼接点云直接宣称为优化地图。原 FAST-LIO 的 `map_save` 依赖发布累积缓存且没有完整检查写盘返回值；当前工作流使用新后端保存接口。

建图与固定地图定位在同一 launch 中互斥：建图由后端发布 `map→odom`，加载固定地图由 GICP 发布。FAST-LIO 保持 `odom→mid360_imu`，不会因回环把局部坐标直接跳变。[统一 launch](../algorithm-review-20261103-session1/worktree/robot_ws/src/robocon_bringup/launch/upper_computer.launch.py)

方法仍有边界：位置先验回环的搜索半径为 1 m，过大的前端漂移可能错过候选；邻域子图自身的畸变也不会被一个刚体回环自动清除。调试中带漂移邻域的 0.4 m 合成闭合误差只能降到约 0.220 m；干净锚帧隔离测试则降到约 0.00106 m。这两者保留在日志中，不能只引用后一个数声称完整建图毫米级。HBA 尚未接入。

### 3.5 FAST-LIVO2：确认已有机制，保留未证实的判断

**已确认。** 当前修补版 `updateState` 在初始化和每轮迭代都清零 `H_sub`，不能把关闭曝光估计后的故障归因于“第七列没初始化”。代码已有光度外点筛选和误差增大回滚，不能再写成“完全没有外点处理”。

**待验证的处理问题。** 光度误差按当轮有效像素数归一化，而有效集合会随投影越界而变化；跨迭代比较不一定在同一批观测上进行。最后一次状态增量通常没有下一轮代价复核。视觉还通过共享协方差更新速度、偏置等状态，几何正确或视觉更新次数多都不能保证最终融合位姿正确。

这些是具体检查位置和可解释的失稳机制，尚不能据此认定运动发散的唯一原因。本轮没有新增外参对照或改相机曝光；融合路线保持原有未通过状态，后续应检查同一观测集合的代价和最终增量接受过程。

## 4. 本轮实际验证结果

| 检查 | 结果 | 能说明什么 |
| --- | --- | --- |
| 应用原生构建 | 9 个 ROS 包完成 | 当前代码可以在该主机编译、安装 |
| Python 回归 | 378 通过，3 跳过 | 既有主控/接口逻辑未出现已覆盖回归 |
| C++ 算法检查 | 8 项 GICP/全局搜索 + 1 项位姿图通过 | 整帧排序、无初值大角度搜索后精配准、重复区域歧义与优化器行为 |
| ROS Action/TF | 通过，连续约 2.46–2.51 Hz | 真实进程的提交、连续匹配恢复、过期证据停止发布 |
| 真实 180.02 s 往返包建图 | 1,797 个 odom，179.60 s 输出；3 关键帧、7,620 个地图点 | 建图、保存和 RViz 数据链可运行；此包没有有效回环 |
| 加载地图后重放 | 1,794 个 odom，179.50 s 输出；自动首锚点 1 次、连续提交 446 次 | 主入口无需人工起点即可建立地图锚点并连续匹配 |
| 配准指标 | 首次内点 RMSE 0.06458 m、重合率 0.99077；末次 0.06832 m、0.97407 | 扫描与保存地图的对应点拟合情况，不是绝对位姿误差 |
| 主控最终状态 | SELF_CHECK，`initial map anchor verified` | 自动初始化已经连到主入口；没有自动 arm 或车辆动作 |
| RViz | 主机显示会话 `:1`，OpenGL 4.6；参考地图和扫描实际显示 | 原生可视化可以使用 |

原始计数、位姿、质量和 TF 保存在 [mapping-evidence.json](../algorithm-review-20261103-session1/mapping-evidence.json)、[localization-evidence.json](../algorithm-review-20261103-session1/localization-evidence.json)，摘要为 [replay-summary.json](../algorithm-review-20261103-session1/replay-summary.json)。旁路观察器的 IMU 接收数不替代 bag 的 SQLite 完整性；覆盖长度取 odom 有效测量时间，未用 `/clock` 启动前的零时间 MissionStatus 计算。

这是同一数据建图后再加载的功能验证，没有独立位姿真值，也没有验证搬到另一个未见视角后的真实成功率。**本轮没有 1 cm 定位精度结论。**

## 5. 查阅方案及采用判断

共保存 11 篇论文及相关官方项目资料，清单与下载哈希见 [sources.json](../algorithm-review-20261103-session1/sources.json) 和 `pgo-sources.json`、`cpu-global-sources.json`。

| 方案 | 核对后的判断 |
| --- | --- |
| FASTLIO2_ROS2 PGO | 已复用核心；其原重定位仍需初值，不能把那个 localizer 直接叫全局定位 |
| 3D-BBS | 已复用 CPU 核心；上游 ROS2 演示使用 GPU，当前 CPU ROS 适配由本项目完成 |
| [GLIM](https://github.com/koide3/glim)、[论文](https://arxiv.org/abs/2407.10344) | 有 CPU 路线和完整建图后端；迁移还涉及依赖与逐点时间输入适配，本轮未替换前端 |
| [KISS-Matcher-SAM](https://github.com/MIT-SPARK/KISS-Matcher/tree/main/ros)、[论文](https://arxiv.org/abs/2409.15615) | 可接外部 LIO 的 ROS2 后端；当前源码要求 CMake 3.24，而主机为 3.22.1，不能直接照文档当成已可运行 |
| [small_gicp](https://github.com/koide3/small_gicp) | 值得用于后续配准加速和信息矩阵检查，但它本身不是全局地点搜索或回环系统 |
| [Scan Context](https://github.com/gisbi-kim/scancontext_tro)、[论文](https://arxiv.org/abs/2109.13494) | 地点检索候选可用；默认 80 m/20 环、排除近 50 帧不适合直接照搬到小场地，描述子也不直接给完整六维位姿 |
| [STD](https://github.com/hku-mars/STD)、[论文](https://arxiv.org/abs/2209.12435) | 三角形描述子可提供几何地点候选；现成演示与工程 ROS2 接口仍需适配，未安装到本轮入口 |
| [HBA](https://github.com/hku-mars/HBA)、[论文](https://arxiv.org/abs/2209.11939) | 针对地图内部一致性优化有价值；需要关键帧与位姿，单张拼接 PCD 不够。当前已保存所需原始层次，但尚未调用 HBA |
| [TEASER++](https://github.com/MIT-SPARK/TEASER-plusplus)、[论文](https://arxiv.org/abs/2001.07715) | 求解可靠对应点下的鲁棒配准，不自动完成地点检索和地图生命周期 |
| [X-ICP](https://arxiv.org/abs/2211.16335) | 可观性应结合约束方向分析；支持前述“点分布协方差不足以代表六维可观性”的方法判断 |

当前没有加入高斯泼溅。建图与重定位采用几何、惯性和位姿图处理；可视化继续使用 RViz。
