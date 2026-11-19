# 固定地图定位：输入、发布权与验证范围

11/01 当前入口已在选择有效 map/calibration bundle 时启用本包的 TF 与连续匹配；主控在 SELF_CHECK 中自动建立首锚点。原生合成验证已通过，实际场地图与定位精度仍未验收，competition 入口仍拒绝。纯雷达地图定位需要的是雷达/机身及前端配置，不以彩色相机外参作为它的输入。完整当前记录见 [11/01 复核](../../../docs/integration-verification-2026-11-01.md)。

`/localization/odom_epoch` 为 `std_msgs/msg/UInt64`，默认必须持续提供（新鲜度门限与 `maximum_scan_age_s` 相同）。审查版工作空间已由 `robocon_localization/lio_session_supervisor` 管理 FAST-LIO 子进程并提供该心跳；统一 launch 在每次启动时持久分配新代次，同时传给 GICP 和主控。启动参数、重定位请求与实时消息必须一致，不能拿 MID 时间监控的 `reset_count` 代替。`require_live_odom_epoch:=false` 只可用于隔离的合成回放。具体接线与 bundle 字段见 [bringup 说明](../robocon_bringup/README.md)。

`/time_alignment/status` 中 `mid360_ptp_clock` 的 `reset_count` 仅用于检测 MID 时间会话变化。首个有效状态建立本进程的时钟基线；之后计数变化、里程计/扫描时间戳明显回退或实时 odom epoch 不匹配都会锁止本进程的地图锚点。恢复须先确认 FAST-LIO 重启与新代次，再重启定位节点并重定位；绝不能把旧 `map→odom` 直接延用。

发布 `map→odom` 的条件是：最近一次成功地图匹配、当前扫描与里程计配对、MID 时间状态及 odom epoch 心跳均在新鲜度窗口内。TF 使用配对扫描时戳，不给旧变换重盖当前时间；失去证据会停止发布并发出拒绝质量。Action 取消与 TF 提交在同一锁下决定，提交后的取消请求会被拒绝。GICP 单次 `align()` 本身不能即时中断，取消/超时最迟在该次调用返回后完成检查。

隔离验证入口：`scripts/smoke_global_localizer.py`。该测试提供合成 epoch 心跳，检查正常提交、连续更新、断流 TF 停止和代次变化后拒绝新请求；它不代表实场地精度与运动安全验收。

## 建图模式

统一入口的 `mapping` 使用同时间戳的 `/cloud_registered_body` 与 `/Odometry`。历史扫描经 CPU 3D-BBS 产生几何候选，GICP 检查重叠、残差、歧义和点面约束方向，通过后交给 GTSAM iSAM2。历史目标不含当前帧和相邻前帧。

`/mapping/save` 等待在途回环，然后以优化位姿重投影关键帧及约 2 Hz 的补充扫描。动态清理以多帧实测自由空间为依据，默认写入 `map.pcd`，原关键帧和优化轨迹仍保存在 `keyframes/` 和 `poses.csv`。RViz 的 `/mapping/map` 在采集时显示优化预览，在保存后发布这份静态地图。固定地图定位从同一 PCD 入口加载。
