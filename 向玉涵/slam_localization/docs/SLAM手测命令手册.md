# ROBOCON 2027 SLAM 手测命令手册

在小主机 `RobopioneerAMD` 的 Ubuntu 22.04、ROS 2 Humble 上执行。项目根目录是 `/home/nvidia/DICodex/ROBOCON2027`。下面用 `20261114_135733` 指现有实场图，测自己的图时换成对应目录名。

| 手测项 | 输入 | 入口 |
| --- | --- | --- |
| RViz 显示点云、轨迹和回环边 | 已保存地图 | §1 |
| 查话题计数和测闭合差 | 已有录包及对应地图 | §2 |
| 建图、录包、回环、动态点清理和两种保存方式 | MID‑360；录包模式加 D435i | §3 |
| 回放建图与回放定位 | 已有录包 | §4、§5 |
| 自动冷启动、持续校正、两次搬移恢复 | MID‑360 或保存的点云 | §5 |
| 无铝片装配外参 | MID‑360 与 D435i 录包 | §6 |
| 视觉重访和 PnP | 已有实场图与包 | §7 |

每个新终端都先执行对应小节的 `cd` 和 `source`。`source` 把对应版本的 ROS 2 包加入当前终端。RViz 从小主机桌面或远程桌面的终端启动。同一次测试用到的几个终端应选择同一个 ROS 域和 `ROS_LOCALHOST_ONLY` 设置。手册的离线回放使用域 179，回归脚本自行使用域 178，现场使用项目环境选用的域。

## 1. RViz 查看已保存地图

在 `config/saved_map_launch.yaml` 设置以下三项。主机当前已选择实场图。

~~~yaml
map_dir: map
map_name: "20261114_135733"
rviz: true
~~~

~~~bash
cd /home/nvidia/DICodex/ROBOCON2027
source env.sh
export ROS_DOMAIN_ID=179 ROS_LOCALHOST_ONLY=1
ros2 launch ./launch/saved_map.launch.py
~~~

灰白是最终静态点云，橙色是 FAST-LIO 轨迹，绿色是优化轨迹，蓝线是接受的回环，红线是保存的首尾关键帧间距。可在左边单独勾选图层，滚轮缩放，中键平移。已有窗口可直接看。关闭 RViz 或启动终端按一次 Ctrl+C 退出。

在另一个终端执行相同的 `cd`、`source` 和 `export` 后，验证迟加入的订阅端也能收到地图。

~~~bash
ros2 topic echo /saved_map/cloud sensor_msgs/msg/PointCloud2 \
  --field width --qos-reliability reliable --qos-durability transient_local --once
~~~

本图应返回 `354469`，RViz 的全局状态应为 `Ok`。其他图按 `map.yaml` 的 `map_points` 核对。

## 2. 查录包和测闭合误差

~~~bash
cd /home/nvidia/DICodex/ROBOCON2027
source env.sh
run="$PWD/map/20261114_135733"
ls -lh "$run/map.pcd" "$run/map.yaml" "$run/poses.csv" "$run/bag/metadata.yaml"
ros2 bag info "$run/bag"
python3 -B resources/tools/map_review/measure_loop.py --map "$run" --bag "$run/bag" --same-pose
~~~

这份包约 528.6 s。检查激光 `/livox/lidar`、IMU、两路雷达时钟、`/tf`、彩色图、RGB CameraInfo 和对齐深度均有消息。主机实查有 5,267 条原始雷达、105,365 条 IMU、15,830 张彩色图和 15,825 张对齐深度。

`--same-pose` 用在现场确认回到同一位置和朝向的跑次。程序用录包首尾稳定窗求位姿差。只测图时使用 `--map "$run"`，只测包时使用 `--bag "$run/bag"`。需要保存结果时运行以下命令。

~~~bash
mkdir -p "$PWD/manual-results"
stamp=$(date +%Y%m%d_%H%M%S)
python3 -B resources/tools/map_review/measure_loop.py --map "$run" --bag "$run/bag" --same-pose \
  --output "$PWD/manual-results/closure-$stamp.json"
~~~

本图与包共匹配 855 个关键帧。停稳三维差为 FAST-LIO **1.29 cm**，图优化位姿补到同一时刻后 **0.98 cm**。直接取保存关键帧的首尾会得到约 19 cm，最后的小幅回位发生在最后一个关键帧之后。复测新图时看两端窗口、XY、三维、带符号的 ΔZ 和姿态差。端点的实际摆位误差和全程绝对误差都未测量。

## 3. 现场建图、录包和保存

1. 接 MID‑360。要同时录 RGB‑D，再接 D435i。先执行 `df -h map`。这次完整包平均约 2.8 GiB 每分钟，当前剩余容量以 `df -h bag` 为准。
2. 在 `config/mapping_launch.yaml` 设置 `map_name` 为尚不存在的目录名，例如 `manual-loop-01`。留空按时间戳生成。`record: true` 会启动相机并把两套传感器录到 `bag/<地图名>`，地图目录的 `bag` 链接到该处，`record: false` 用于纯雷达建图。`rviz: true` 开启实时建图视图。
3. 起点和终点都在里程计已出数据时停稳几秒，并对准同一位置与朝向。测回环时走一段完整路线再重访，经过有辨识度的墙角、柱子或门框。

~~~bash
cd /home/nvidia/DICodex/ROBOCON2027
source env.sh
nano config/mapping_launch.yaml
ros2 launch ./launch/mapping.launch.py
~~~

终端会打印 `Mapping output ...`，RViz 应看到实时点云、优化地图和轨迹。另一终端执行相同的 `cd` 和 `source`，检查输入与里程计。

~~~bash
timeout 8s ros2 topic hz /livox/lidar
timeout 8s ros2 topic hz /livox/imu
ros2 topic echo /Odometry nav_msgs/msg/Odometry --qos-reliability best_effort --once
timeout 8s ros2 run tf2_ros tf2_echo odom mid360_imu
~~~

录包模式下检查相机。

~~~bash
timeout 8s ros2 topic hz /camera/camera/color/image_raw
timeout 8s ros2 topic hz /camera/camera/aligned_depth_to_color/image_raw
ros2 topic echo /camera/camera/color/camera_info sensor_msgs/msg/CameraInfo --qos-reliability best_effort --once
~~~

**运行中保存。** 在第二终端执行下列服务。预期返回 `success: true` 和输出目录。

~~~bash
ros2 service call /mapping/save std_srvs/srv/Trigger "{}"
~~~

**自动保存。** 用一个新目录单独试验。在终点停稳后按一次 Ctrl+C，等待退出，再用实际名字检查结果。

~~~bash
run="$PWD/map/manual-loop-01"
for f in map.pcd map.yaml poses.csv; do test -s "$run/$f" || echo "缺少 $f"; done
find "$run/keyframes" -maxdepth 1 -name '*.pcd' | wc -l
cat "$run/map.yaml"
ros2 bag info "$run/bag"
~~~

- **建图和回环**。PCD 能在 §1 打开，关键帧数与 YAML 一致，轨迹在图上连续。`Loop accepted` 或 `loops` 大于 0 表示回环入图。长路线出现 `global_search_timeout` 时，把原始数字留在报告里。
- **同步录包**。`record: true` 的目录有 `bag/metadata.yaml`，相机、雷达、里程计 TF 话题有计数。纯雷达模式以地图产物和实时话题验收。
- **动态点清理**。在可见墙面前移动一个纸箱，挪开后从几个视角继续扫背景。先在实时图确认纸箱可见，再打开最终 PCD 检查当时纸箱位置和原背景。`removed_dynamic_returns` 是算法删除量，实际删留通过场景比对确认。

## 4. 无传感器重放建图

小主机已有一份约 40 秒的短包，可测试 FAST-LIO 的重放和保存。要跑完整闭环，再把输入改成实场长包。

~~~bash
cd /home/nvidia/DICodex/ROBOCON2027
export ROS_DOMAIN_ID=179 ROS_LOCALHOST_ONLY=1
stamp=$(date +%Y%m%d_%H%M%S)
bash run_slam.sh mapping \
  --bag "$PWD/bag/legacy/robocon2027/artifacts/fusion-20261017-bag-v2/bag" \
  --output "$PWD/map/replay-manual-$stamp" --rviz
~~~

结束后看到 `Map saved to ...`，检查新目录的 PCD、YAML 和轨迹。重放入口给求解器送雷达、IMU 和时钟话题。相机内容用 §2 或 §6 检查。

## 5. 固定地图定位、持续校正和恢复

### 无传感器自检

这个入口运行实际的 ROS Action、连续 GICP 和 TF 节点，通过注入几何点云检查冷启动和两次恢复。两条命令依次运行。

~~~bash
cd /home/nvidia/DICodex/ROBOCON2027
source env.sh
export TMPDIR="$PWD/resources/tmp"
mkdir -p "$TMPDIR"
python3 -B resources/tests/localization/smoke_auto_relocalize.py
python3 -B resources/tests/localization/verify_recorded_recovery.py \
  "$PWD/map/20261114_135733" 193 533 767
~~~

第一条使用有已知位姿的合成场景，第二条使用建图时保存的实测扫描，模拟三个不同地点和视角。检查 `anchor 1`、`anchor 2`、`anchor 3`。脚本会核对 TF、每个锚点的 `static_map` 连续提交、失配时的箭头撤下及恢复后的状态。三处实测扫描相对建图中保存的位姿差分别为 0.16、3.66、2.63 cm。

### 现场手测

结束建图，接通 MID‑360。在 `config/localization_launch.yaml` 填所在场地保存的地图名和 `rviz: true`。`config` 与 `livox_config` 留空时优先用地图内的参数快照。

~~~bash
cd /home/nvidia/DICodex/ROBOCON2027
source env.sh
ros2 pkg prefix robocon_global_localization
nano config/localization_launch.yaml
ros2 launch ./launch/localization.launch.py
~~~

包路径应为 `install/robocon_global_localization`。在其他终端载入同样环境，分别观察质量与 TF。

~~~bash
ros2 topic echo /localization/quality robocon_interfaces/msg/LocalizationQuality
~~~

~~~bash
ros2 run tf2_ros tf2_echo map mid360_imu
~~~

1. **冷启动**。在有墙角或门框的位置停稳。进入 `TRACKING`，收到 `source: relocalization`、`accepted: true`、`tf_committed: true` 后，对照 RViz 里实时点云与固定图。
2. **持续校正**。缓慢移动、转向，观察 `source: static_map` 反复提交。`localization_generation` 保持，`tf_revision` 继续增加。
3. **两次找回**。在同一会话做两次不同位置和朝向的搬移。每次先用临时遮挡或移到地图外，让点云持续失配，看到追踪箭头消失。再在新位置恢复视野、停稳。观察 `LOST` 或 `RELOCALIZING`，然后回 `TRACKING`，每次锚点代次增加。测试期间维持雷达供电和网络。
4. **歧义和超时**。在对称或几何不足的位置观察 `rejection_reason` 与重试，再移到有不同结构的视角验证恢复。看当前点云和地图的空间对应关系。几何重复的区域可能匹配到相似位置，现场应对照实际摆放点。雷达时钟或里程计会话代次改变时，重新启动定位会话。

按 Ctrl+C 退出。最后记录位姿箭头方向、实时点云对齐、两次新锚点和各次失败原因。只打开 §1 的静态预览时，用蓝线看已存回环；定位箭头在本节的窗口。

### 录包重放固定地图定位

~~~bash
cd /home/nvidia/DICodex/ROBOCON2027
source env.sh
export ROS_DOMAIN_ID=179 ROS_LOCALHOST_ONLY=1
python3 -B scripts/launch_slam.py localization \
  --map "$PWD/map/20261114_135733" --bag "$PWD/map/20261114_135733/bag" --rviz
~~~

按实时速率约 9 分钟，检查 `/localization/quality` 的定位锚点、校正提交与拒绝原因。需缩短回放时可用已有的短包和它对应的地图。选择其他场地的包时先检查地图和录包的对应关系。

## 6. 无铝片手持架的空间外参

先用已有包验证读包和报告。早期与返回各取一段静止窗。

~~~bash
cd /home/nvidia/DICodex/ROBOCON2027
source env.sh
tool="$PWD/resources/rig/calibration-local"
bag="$PWD/map/20261114_135733/bag"
stamp=$(date +%Y%m%d_%H%M%S)
out="$PWD/manual-results/calibration-$stamp"
python3 -B "$tool/extract_scene.py" "$bag" --list
python3 -B "$tool/extract_scene.py" "$bag" --at 8.30 --output "$out/early.npz"
python3 -B "$tool/extract_scene.py" "$bag" --at 467.30 --output "$out/return.npz"
python3 -B "$tool/calibrate.py" --diagnostic \
  --fit "$out/early.npz" --holdout "$out/return.npz" --output "$out/diagnostic"
~~~

应读出 640×480 RGB‑D，生成 `report.json` 与 `holdout-01.png`。这两段是同一机位，报告预期给出 `diagnostic_or_rejected`。下面的 Humble 自检可以确认读包和求解器对已知外参的输出。

~~~bash
python3 -B "$tool/_work/selfcheck.py" "$out/synthetic"
~~~

实测标定沿用无铝片且雷达相机刚性固定的装配。用 §3 录不同朝向的平面和墙角，每个视角停稳至少 2 秒。用 `extract_scene.py --at 秒数` 分别生成三个求解窗 `fit1.npz` 至 `fit3.npz` 和两个留出窗 `hold1.npz`、`hold2.npz`。

~~~bash
python3 -B "$tool/calibrate.py" \
  --fit "$out/fit1.npz" "$out/fit2.npz" "$out/fit3.npz" \
  --holdout "$out/hold1.npz" "$out/hold2.npz" --output "$out/result"
~~~

外参的方向是 `p_camera_color_optical = Rcl @ p_mid360 + Pcl`，平移单位米。观察留出图的投影边界和报告中的覆盖、多初值一致性。通过筛选才会写 `candidate-Rcl-Pcl.json`，之后再结合实装与运动回放验算。静止流程估计的是空间外参。

## 7. 视觉重访的离线诊断

~~~bash
cd /home/nvidia/DICodex/ROBOCON2027
source env.sh
study="$PWD/resources/studies/map-study-20261116"
run="$PWD/map/20261114_135733"
python3 -B "$study/probe_visual.py" "$run"
python3 -B "$study/inspect_visual_coverage.py" "$run"
~~~

输出写到该研究目录的 `visual-probes.json` 和 `visual-coverage.json`。已知起点和返程的 SIFT 配对得到 64 个 PnP 内点，重投影误差中位数约 0.23 像素。沿路线的反向重访经常缺少同向图像。正式定位启动中的视觉种子和持续视觉纠偏还在开发阶段。

命令行为参见 [ROS 2 Humble rosbag2](https://github.com/ros2/rosbag2/tree/humble) 和 [RViz Humble](https://github.com/ros2/rviz/tree/humble)。
