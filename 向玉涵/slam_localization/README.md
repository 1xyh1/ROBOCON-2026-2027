# ROBOCON 2027 SLAM 与定位算法源码包

整理日期：2026-11-19
来源：`D:\DICodex\ROBOCON2027` 中各阶段源码 / 验证快照（对应 Ubuntu 小主机统一工作空间 `/home/nvidia/DICodex/ROBOCON2027` 的 Windows 侧同步副本）。
整理方式：**只读复制**。源目录未做写入、移动或内容修改；本包内被复制的文件与源快照逐字节一致（`diff -r` 校验通过）。

> 本包只包含算法源码、运行入口、参数、补丁和整理后的说明，不包含录包（`bag/`）、地图（`map/`）、标定原始数据、验证证据等大文件。要恢复完整可编译工作空间，请从 Ubuntu 主机同步完整 `src/`、`resources/runtime` 和驱动包。

## 1. 算法组成

| 环节 | 实现 | 位置 | 说明 |
| --- | --- | --- | --- |
| SLAM 前端（LiDAR-惯性里程计） | FAST-LIO2（ROS 2 移植）+ MID-360 内置 IMU | `src/fast_lio` | IMU 传播、逐点去畸变、迭代误差状态卡尔曼滤波、点到面残差、ikd-Tree 局部地图；发布 `odom → mid360_imu`、`/Odometry`、`/cloud_registered_body`。部署版含本项目补丁（帧名/TF 可配置、MID-360 扫描时间戳乱序容错、协方差等，见 `patches/`）。 |
| SLAM 后端（回环建图） | CPU 3D-BBS 回环检索 + GICP 验证 + GTSAM iSAM2 位姿图 | `src/robocon_global_localization` | 输入同一时间戳的 `/cloud_registered_body` 与 `/Odometry`；按 0.5 m / 10° 提取关键帧；历史扫描经 CPU 3D-BBS 产生几何候选，整帧 GICP 检查重合/残差/歧义/点到面可观性，通过后以 Huber 回环因子加入 iSAM2。保存时按优化位姿重投影关键帧并做动态点清理，输出 `map.pcd`、`map.yaml`、`poses.csv`、`keyframes/`。 |
| 无初值全局重定位 | CPU 3D-BBS 分支定界全图搜索 + GICP 精配准 | `src/robocon_global_localization`（`global_localizer_node`、`global_search`） | `Relocalize.action` 空 seeds 表示全图搜索；重力对齐后搜索 XYZ 与完整航向，保留分数接近的不同候选区域交给 GICP 歧义比较；搜索结束后取当前扫描并用搜索前后 LIO 相对运动传播初值，再精配准提交首锚点。 |
| 固定地图连续定位 | PCL Generalized-ICP 连续匹配 | `src/robocon_global_localization`（`gicp_matcher`） | 按整帧截断代价排序（未匹配点也计入代价，避免只看内点 RMSE 的选解偏差），发布 `map → odom` 与 `/localization/quality`。 |
| 自动定位 / 恢复 / LIO 会话 | 自动首锚点状态机、LIO 质量门、FAST-LIO 子进程监督 | `src/robocon_localization` | `auto_relocalize_node.py`：SELF_CHECK 自动请求重定位、失锁后恢复；`lio_quality_node.py`：LIO 质量门；`lio_session_supervisor.py` + `odom_session.py`：管理 FAST-LIO 子进程并发布 `/localization/odom_epoch` 心跳/代次，防止旧地图锚点被复用。 |
| 启动接线 | 统一建图 / 定位入口 | `src/robocon_bringup` | `upper_computer.launch.py` 负责建图与固定地图定位的互斥 `map → odom` 发布权、LIO/定位节点组装、RViz 视图与运行 profile。 |
| 时间对齐 | 雷达 / 相机时基处理 | `src/robocon_time_alignment` | D435i 时间对齐、MID-360 时钟查询与时间戳监视；定位门控会使用 `/time_alignment/status` 中的 MID-360 PTP 时钟状态。 |
| 视觉-惯性融合（实验） | FAST-LIVO2 | `src/experimental/fast_livo`、`src/robocon_fusion` | FAST-LIVO2 源码（上游 commit 见 `.robocon-source.json`）与本项目补丁（`patches/fusion/`）；`robocon_fusion` 为融合回放 / bench 接口与诊断。**运动融合问题仍未修好，未接入正式定位**。 |
| 公共 Python 库 | 算法基准、时间对齐、配准、运行时资产等 | `src/robocon_core/robocon_core` | 被 launch / 工具直接引用（如 `runtime_assets.py`）；不含 Windows 快照未保存的 ament 元数据（`package.xml` / `setup.py`），Ubuntu 主机上有完整包。 |
| 自定义接口 | Action / Msg / Srv | `src/robocon_interfaces` | `Relocalize.action`、`LocalizationQuality.msg`、`TimeAlignmentStatus.msg` 等，算法包编译依赖。 |

方法细节、审查结论与实测记录见 `docs/algorithm-method-review-2026-11-03.md`；现场命令见 `docs/SLAM手测命令手册.md`。

## 2. 目录结构

```text
slam_localization/
├── README.md                    本说明
├── src/
│   ├── fast_lio/                SLAM 前端：FAST-LIO2（ROS 2，含部署补丁）
│   ├── robocon_global_localization/   建图回环后端 + 全局重定位（含 third_party/3d_bbs、fastlio2_ros2_pgo）
│   ├── robocon_localization/    自动定位、恢复、LIO 会话管理
│   ├── robocon_bringup/         统一启动接线与 RViz 配置
│   ├── robocon_time_alignment/  雷达 / 相机时间对齐
│   ├── robocon_core/robocon_core/     公共 Python 模块
│   ├── robocon_interfaces/      Action / Msg / Srv 定义
│   ├── robocon_fusion/          融合接口、回放与诊断（实验）
│   └── experimental/
│       ├── fast_livo/           FAST-LIVO2 源码（实验，未接入正式定位）
│       └── vikit_common/        包元数据（完整源码由 patches/fusion/ 处理，主机侧保存）
├── launch/                      建图、定位、保存地图预览入口（mapping / localization / saved_map）
├── config/                      MID-360、D435i、运行选择与手持架标定参数
├── patches/                     FAST-LIO / Livox 补丁与 FAST-LIVO2 实验补丁
├── tools/                       回放、融合构建、定位恢复自检脚本
└── docs/                        算法方法审查 + SLAM 手测命令手册
```

## 3. 来源与版本（按包）

| 本包路径 | 源快照（相对 `D:\DICodex\ROBOCON2027`） | 快照日期 |
| --- | --- | --- |
| `src/fast_lio` | `host-pull-20261031-221900/files/robocon2027/robot_ws/src/fast_lio`（部署版，已打 `patches/` 中补丁） | 2026-10-31 |
| `src/robocon_global_localization` | `localization-validation-20261117/src/robocon_global_localization`（含 11/17 缓存与恢复修复） | 2026-11-17 |
| `src/robocon_localization` | `relocalization-20261112-session1/worktree/robot_ws/src/robocon_localization` | 2026-11-12 |
| `src/robocon_bringup` | `relocalization-20261112-session1/worktree/robot_ws/src/robocon_bringup` | 2026-11-12 |
| `src/robocon_interfaces` | `relocalization-20261112-session1/worktree/robot_ws/src/robocon_interfaces` | 2026-11-12 |
| `src/robocon_time_alignment` | `relocalization-20261112-session1/worktree/robot_ws/src/robocon_time_alignment` | 2026-11-12 |
| `src/robocon_core/robocon_core` | `relocalization-20261112-session1/worktree/src/robocon_core` | 2026-11-12 |
| `src/robocon_fusion` | `src/robocon_fusion`（根目录当前的融合接口/回放包） | 2026-11-19 |
| `src/experimental/fast_livo` | `src/experimental/fast_livo`（FAST-LIVO2 源码，含近期修改） | 2026-11-19 |
| `src/experimental/vikit_common` | `src/experimental/vikit_common` | 2026-11-19 |
| `launch/`、`config/` | 根目录 `launch/`、`config/` | 2026-11-18 |
| `patches/` | `relocalization-20261112-session1/worktree/patches` | 2026-11-12 |
| `tools/` | 根目录 `scripts/` + `localization-validation-20261117/` 的定位恢复自检脚本 | 2026-11-19 |
| `docs/` | `docs/algorithm-method-review-2026-11-03.md`、`docs/SLAM手测命令手册.md` | 2026-11-03 / 2026-11-18 |

说明：Windows 快照是按阶段从主机回传的记录，不同包的最新时间不同；上表取每个包在 Windows 侧能取到的最新版本组合。Ubuntu 主机 `/home/nvidia/DICodex/ROBOCON2027` 才是当前完整工作空间。

## 4. 构建

目标环境：Ubuntu 22.04 + ROS 2 Humble（项目实测环境为小主机 `RobopioneerAMD`）。

主要依赖：

- 通用：`ament_cmake` / `ament_python`、`rclcpp`、`pcl_ros`、PCL 1.12、Eigen3、OpenMP、`tf2_ros`、`launch_ros`；
- `src/fast_lio`：`livox_ros_driver2`（驱动，不在本包内）、PCL、Eigen、OpenMP；
- `src/robocon_global_localization`：**GTSAM**（iSAM2）、PCL、OpenMP、`robocon_interfaces`；`third_party/3d_bbs` 与 `third_party/fastlio2_ros2_pgo` 已内嵌源码，不需要额外下载；
- `src/experimental/fast_livo`：`cv_bridge`、`image_transport`、OpenCV、Sophus、fmt、`vikit_common`、`livox_ros_driver2`；
- 录包 / 相机相关：`realsense2_camera`、`rosbag2`（驱动不在本包内）。

示例（在完整工作空间内，包名按需增删）：

```bash
cd /home/nvidia/DICodex/ROBOCON2027
source env.sh
colcon build --symlink-install --packages-up-to \
  fast_lio livox_ros_driver2 robocon_interfaces \
  robocon_global_localization robocon_localization robocon_bringup \
  robocon_time_alignment robocon_fusion
source install/local_setup.bash
```

注意：`src/robocon_core` 在本快照中只有 Python 模块、没有 ament 元数据，需以主机上的完整包为准；本包要独立编译时，还需补回 `livox_ros_driver2`、`realsense2_*` 等驱动包，以及 `vikit_common` 的完整源码。

## 5. 运行

启动入口在项目根目录，按 `config/` 中的 YAML 选择地图、传感器和 RViz：

```bash
# 建图（可选录包、RViz）
ros2 launch ./launch/mapping.launch.py

# 加载已保存地图做固定地图定位
ros2 launch ./launch/localization.launch.py

# 只看已保存地图（静态点云 / 优化轨迹 / 回环）
ros2 launch ./launch/saved_map.launch.py
```

- 建图参数：`config/mapping_launch.yaml` + `config/mid360_b3.yaml` + `config/livox_mid360_mini_host.json`；
- 定位参数：`config/localization_launch.yaml`（`map_name` 选地图目录，`config` / `livox_config` 留空时使用地图内保存的参数快照）；
- 保存地图预览：`config/saved_map_launch.yaml`；
- 无传感器回放、闭合差测量、两次搬移恢复的手测步骤见 `docs/SLAM手测命令手册.md` §4、§5。

提示：`launch/` 与 `tools/` 中的脚本按 Ubuntu 项目根布局（`config/`、`map/`、`bag/`、`resources/`）编写，脚本里的绝对路径、IP 和日期是历史阶段记录，换机器使用前需要按实际路径调整。

## 6. 验证状态（项目记录，非本包复测）

以下结论摘自 `D:\DICodex\ROBOCON2027` 中的项目记录，用于说明算法当时的状态；本包未重新执行构建或实车测试。

- 2026-11-03：9 个 ROS 包原生构建通过；378 项 Python 通过 / 3 项跳过；8 项 GICP/全局搜索 + 1 项位姿图 C++ 检查通过；ROS Action/TF 联调通过。真实 180.02 s 往返录包完成建图（3 关键帧、7,620 个地图点）、保存、重新加载、自动首锚点 1 次、连续提交 446 次。该阶段**没有 1 cm 定位精度结论**。
- 2026-11-07：真实 297.68 s 运动录包 10 条回环入图，静态 PCD 40,464 点并用定位节点回读；保存地图引入多帧自由空间的动态点清理。
- 2026-11-14 ~ 11-17：实场图 `20261114_135733` 的三维闭合差为 FAST-LIO 1.29 cm、图优化轨迹补到相同端点后 0.98 cm；三处实测扫描的恢复位姿差 0.16 / 3.66 / 2.63 cm；11-17 修复定位缓存与恢复流程。
- 2026-11-18：迁移后的统一 ROS 2 工作空间 16 包原生构建通过，GICP 与图优化两个 C++ 测试、合成与实测点云自动恢复、短录包回放建图和保存地图 RViz 复核通过。

未闭合项：实场任意位置的定位成功率与独立位姿真值尚未验收；FAST-LIVO2 运动融合仍失稳，保持实验状态；建图回环的大漂移邻域、HBA 等仍是已知边界。

## 7. 未包含内容

- 录包 `bag/`、地图 `map/`、`resources/` 下的标定原始数据、验证证据与日志；
- 驱动包：`livox_ros_driver2`、`realsense2_camera`、`realsense2_camera_msgs`、`realsense2_description`、`Livox-SDK2`；
- 非算法包：`robocon_navbot`（仿真）、`robocon_executive`（主控）、`robocon_perception`（感知）、`cdc_keyboard` 等；
- FAST-LIO 上游 `doc/` 媒体、`Log/`、`PCD/`、`.github/`（文档图片与运行输出，非算法代码）；
- `src/fast_lio/include/ikd-Tree/.git` 子模块指针文件（已移除以便作为普通文件提交，源码文件完整保留）。

## 8. 上游与许可

- FAST-LIO（`src/fast_lio`）：上游 [hku-mars/FAST_LIO](https://github.com/hku-mars/FAST_LIO)，ROS 2 移植 fork（README 标注 maintainer Ericsiii）；保留原 `LICENSE`。本项目补丁见 `patches/fast_lio_ros2_*.patch`。
- FAST-LIVO2（`src/experimental/fast_livo`）：上游 commit 记录在 `.robocon-source.json`（`837b7bbc1431cb04cf936528e52c83c835efba8e`），保留原 `LICENSE` 与 `patches/fusion/` 补丁。
- 3D-BBS（`src/robocon_global_localization/third_party/3d_bbs`）：[KOKIAOKI/3d_bbs](https://github.com/KOKIAOKI/3d_bbs) revision `41529a34a2fb9618b5ff560fb3c2363f1615666d`，MIT；仅保留 CPU 分支及必要头文件。
- fastlio2_ros2_pgo（`third_party/fastlio2_ros2_pgo`）：[liangheming/FASTLIO2_ROS2](https://github.com/liangheming/FASTLIO2_ROS2) revision `7baa59f7339ba88ce3b646fe35b23f61031c5d3f`，MIT；仅位姿图核心，ROS 节点为本项目适配。
- `robocon_*` 包为本项目团队代码。

使用或再分发前请遵守各上游许可证（FAST-LIO / FAST-LIVO2 为 GPL-2.0，`LICENSE` 已随目录保留；BSD/MIT 组件见各自目录）。
