# 启动入口与定位数据流（2026-11-01 审查版）

当前已验证的应用工作空间在 DICodex 的 `verification-20261101-session2/worktree`；小主机通过本轮根目录 `run_runtime.sh` 加载九包 overlay 和独立 core，再执行 `scripts/launch_runtime.py`。原硬件驱动与 FAST-LIO 二进制继续作为 underlay。最新结果和未闭合项见 [原生复核](../../../docs/integration-verification-2026-11-01.md)。

2026-11-03 已在小主机部署 RViz2：`view:=lio|map|mapping|fusion`。`run_slam.sh mapping` 启动回环建图与地图保存，`run_slam.sh localization` 加载保存地图并自动完成全局初始化；`--rviz` 同时打开对应视图。建图与固定地图匹配在同一 launch 中互斥，不会同时取得 `map→odom` 发布权。真实录包已完成建图、加载、自动首锚点和连续定位。操作见 [RViz 使用说明](../../../docs/rviz-2026-11-03.md)。

`launch_runtime.py --profile ... --dry-run` 只解析并显示启动命令。真实启动仍先核对运行 profile、Livox 网络参数和硬件。`sensors.d435i.enabled: false` 时只要求 MID 时基，不启动相机适配/感知节点，也不让主控等待 D435i 心跳。省略该键时沿用原有双传感器模式。

启用固定地图定位时，`localization.enabled: true` 必须同时指向已有协议下的已批准 map 与 calibration bundle。除了原有版本、身份、审阅证据和数据文件哈希，map manifest 还要明确选择实际输入：

```yaml
pcd_file: surveyed-floor.pcd
floor: ground
```

全局搜索无需预设起点。PCD 必须列入该 bundle 的 `files` 哈希清单；calibration manifest 的 `fast_lio_config` 同样必须选中其已列入哈希清单的 FAST-LIO ROS 参数 YAML。相机外参不是这条纯雷达链路的输入。直接 SLAM 入口也可读取本项目建图目录，按实际地图文件和前端配置哈希建立本次运行身份；该配置身份不冒充实测整车或相机标定。

统一入口据此生成运行目录的 `localization-runtime.yaml`，同时传入地图匹配节点和主控；选中的 FAST-LIO 参数文件也真正传入前端。地图 ID、配置/标定 ID、楼层均由实际节点消费。现有车辆 `practice.yaml` 没有签收比赛场地 bundle；已交付的 SLAM 入口读取建图产物即可运行定位，不要求伪造资产 approved 标志。

前端启动时，launch 在 checkpoint 目录持久分配递增 `odom_epoch`。`lio_session_supervisor` 管理 FAST-LIO 子进程并在存活期间发布 `/localization/odom_epoch`；GICP 和主控使用同一个分配值。前端退出会结束整组节点，没有自动重启或沿用旧地图锚点。再次启动必须取得新代次。

固定地图链的公开接口顺序是：启动输入与节点 → SELF_CHECK 等到新鲜时基/局部质量后以空种子请求 `/localization/relocalize` → CPU 3D-BBS 全图搜索、GICP 精配准 → Action 提交与匹配代次质量建立首锚点 → GICP 连续更新 `map→odom`。车辆主控后续的 prepare/arm/开始与停止确认另有既有契约；首锚点成功不会自动运动。建图模式不启动这条主控执行链。

当前入口未接独立 CDC 工作空间。该工作空间已有底盘 `/cmd_vel` 接收链和 `/leg/lift` Action server；缺的是本入口的导航/腿任务适配、全局取消请求处理与 `/mission/cancel_ack` 硬件确认生产端，夹取链也未完整实现。`robocon_navbot` 是独立仿真，`ObserveTarget.action` 目前只有接口定义。不能将“底层驱动已有”和“主控已经能执行任务”混为一项。

11/01 从本地备份补审发现 CDC 残帧解析、PC 速度约束、各设备反馈新鲜度、跨线程共享状态及 TF 发布权尚有确定问题。轮式配置会发布 `odom→base_link`，而雷达发布 `odom→mid360_imu`，两者尚未通过实测机身安装变换统一；不能直接同时启用并补单位变换。Lift 取消保留旧目标，也不能拿 CANCELED 生成物理停止确认。详细源证据和实际 C++ 反例在审查目录 `backup-review-20261101/review.md`。

残帧和 PC 速度限制修复已在本轮 `cdc-worktree` 原生构建，并通过协议、真实仲裁节点和 PTY 整机软件验证；正式驱动未替换。其它执行反馈/停止/TF 问题仍然打开，不因局部修复就启用实车执行。

接口联调使用实际安装的 GICP、质量节点、主控、D435i 适配与感知节点；传感器输入和估计器由显式测试夹具代替，未启动硬件或执行器。联调通过只能证明对应消息、Action、Service、TF 和会话生命周期，不证明定位精度、硬件时序或任务执行能力。
