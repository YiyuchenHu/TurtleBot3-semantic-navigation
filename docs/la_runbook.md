# LocateAnything 模式运行手册（la_runbook）

> 只想知道怎么跑：看 [quickstart.md](quickstart.md)（一页命令）。本文是细节与坑。

手动开终端跑通 LocateAnything 语义导航栈并录屏的操作手册。每个终端一节，命令逐条可直接粘贴，**只用 docker / source / ros2 / ffmpeg 原语**（R07 起；仓库脚本降级为 §12 可选封装）。数据（预期耗时、预期日志行）全部来自 R02b/R02c 验收实测。

宿主机前提：NVIDIA 驱动 + NVIDIA Container Toolkit + X11 桌面；仓库位于
`~/Desktop/Github_Project/TurtleBot3-semantic-navigation`（下文简写 `$REPO`）。

---

## 1. 总览与架构

一次完整演示 = 容器（docker，§3）+ 分层起栈（六层：Gazebo / Nav2 / 感知链 / 探索 / 查询-协调 / RViz，每层一条 launch 或逐节点 `ros2 run`，§4）+ 观察（ros2 node/topic/param，§5）+ 发命令（ros2 topic pub，§6）。全部是 docker 与 ros2 原语，不依赖本仓库的脚本；本仓库的 launch 文件只在 §4.8 作为可选一键版，脚本只在 §12 作为可选封装列出。

```mermaid
flowchart LR
    subgraph grounding 容器（GPU, LocateAnything-3B）
        srv[HTTP server<br>POST /detect]
    end
    subgraph sim 容器（ROS 2 Humble + Gazebo Classic）
        cam([/camera/image_raw]) --> det[la_detector_node<br>词表 + coverage 类]
        det -- HTTP --> srv
        det -- /detector_node/detections --> loc[localizer<br>像素→bearing + LiDAR 距离]
        loc --> mem[semantic_memory<br>观测聚合 / 晋升门槛]
        mem --> mapmem[semantic_map_memory<br>占据栅格校验 → landmark]
        mapmem --> qry[semantic_query<br>规则解析 + 搜索建议]
        usr([/user_command<br>终端 C]) --> coord[coordinator 状态机<br>EXPLORING/SEARCHING/SEMANTIC_NAV]
        coord --> qry
        qry --> coord
        coord --> nav[nav_adapter → Nav2<br>NavigateToPose]
        coord -. "/active_search 注入点<br>（词表注入 + 晋升门槛 5）" .-> det
        coord -. "/active_search" .-> mem
    end
```

搜索链路一句话：查询词不在 memory 时，coordinator 把该词经 `/active_search` 注入
la_detector 的 detect 词表（最高去重优先级）并开探索；semantic_memory 对**词表外的**
搜索类把晋升门槛降到 3；landmark 出现后 coordinator 暂停探索、重发原命令走正常导航链。
R05 起：探索干涸后不直接放弃，先跑 4 轮**巡逻扫视**（§7.3）；与搜索词同族的基础词条
在搜索期**临时挂起**，免得通用词把细粒度表达式票数压死。

## 2. 前置检查

```bash
cd ~/Desktop/Github_Project/TurtleBot3-semantic-navigation
docker images | grep tb3-semantic-nav        # 应有 sim 与 grounding 两个镜像
docker volume ls | grep docker_hf_cache      # LocateAnything-3B 权重所在卷
```

若改过 `src/` 下任何 launch/config/代码，先在 sim 容器里重建（跳到 §3 起容器后执行）：

```bash
docker compose -f docker/compose.yaml -f docker/compose.local.yaml exec sim \
  bash -c "cd ~/ws && ./build.sh"
```

重命名/删除过 config 文件时 colcon 会因旧 build 目录里的残留引用失败（R07 改名
nav2_params_la.yaml 时踩到），先 `rm -rf build/<pkg> install/<pkg>` 再 build.sh。

## 3. 终端 A — 起容器（docker）

只用 docker / docker compose 原语。两个容器：`docker-sim-1`（ROS 2 Humble + Gazebo + Nav2）、
`docker-grounding-1`（LocateAnything-3B HTTP 服务，与 sim 共享网络命名空间）。

```bash
cd ~/Desktop/Github_Project/TurtleBot3-semantic-navigation
xhost +local:docker                                   # 容器里的 Gazebo/RViz 要画到你的 X 屏幕
docker compose -f docker/compose.yaml -f docker/compose.local.yaml up -d
docker compose -f docker/compose.yaml -f docker/compose.local.yaml ps       # 两个都 Up
docker exec docker-sim-1 nvidia-smi -L                # 必须列出 "GPU 0"
docker logs -f docker-grounding-1                     # 看权重加载；冷启动 ~76 s，热启动 ~10 s；Ctrl+C 退出跟随
curl http://127.0.0.1:8801/health                     # {"status":"ok", ..., "device":"cuda", ...}
```

`nvidia-smi` 报 `Failed to initialize NVML: Unknown Error` = 容器丢了 GPU（§10.8）。修：

```bash
docker compose -f docker/compose.yaml -f docker/compose.local.yaml restart sim
docker compose -f docker/compose.yaml -f docker/compose.local.yaml restart grounding   # 必须跟着重启（共享 netns）
```

改过 `src/` 后重建（colcon 在容器里）：

```bash
docker exec -it docker-sim-1 bash -c "cd ~/ws && source /opt/ros/humble/setup.bash && colcon build --symlink-install"
```

重命名/删除过 config 文件时先 `rm -rf build/<pkg> install/<pkg>` 再 build（旧 build 目录残留引用）。

## 4. 逐节点手动起栈（每个节点一个终端）

不用本仓库的 `full_semantic_nav_la.launch.py`。栈里一共 6 层，每层一个终端；每个终端都先做同样的
三行"进容器 + source"：

```bash
docker exec -it docker-sim-1 bash                   # 进容器（-i 交互，-t 分配终端）；退出用 exit
source /opt/ros/humble/setup.bash                   # ROS 2 底座
source ~/ws/install/setup.bash                      # 本工作空间 overlay
```

`docker exec` 每次都是在**同一个已运行的容器**里开一个新 shell，所以任意多个终端都能看到同一套
ROS 图；容器里 `TURTLEBOT3_MODEL=waffle_pi` 已经设好（compose）。下面每个终端起完就别动它，
`Ctrl+C` 就是关那个节点。

### 4.1 终端 1 — Gazebo 世界 + 机器人（一条 launch，参数选世界与起点）

```bash
ros2 launch tb3_frontier_exploration gazebo_sim.launch.py                       # 默认：person world，起点 (1, -1)
ros2 launch tb3_frontier_exploration gazebo_sim.launch.py --show-args           # 看全部参数
ros2 launch tb3_frontier_exploration gazebo_sim.launch.py world:=warehouse_models x_pose:=-1.2 y_pose:=-1.2
ros2 launch tb3_frontier_exploration gazebo_sim.launch.py world:=/abs/path/other.world gui:=false
```

参数：`world`（别名 `warehouse_models_person` / `warehouse_models` / `detector_test`，或 .world 绝对路径）、
`x_pose` `y_pose`（起点，§9 规则）、`gui`（false = 不开 gzclient）、`use_sim_time`。
它里面就是四个官方 launch（gazebo_ros 的 gzserver / gzclient，turtlebot3_gazebo 的
robot_state_publisher / spawn_turtlebot3）加一行把仓库的 `models/`（彩衣人）追加到 `GAZEBO_MODEL_PATH`；
文件 `src/tb3_frontier_exploration/launch/gazebo_sim.launch.py`，70 行，可以照着改。

核对：`ros2 service list | grep spawn`（/spawn_entity 出现 = gzserver 真正就绪）、`ros2 topic hz /scan`、
`ros2 topic hz /camera/image_raw`。
spawn 报 `Service /spawn_entity unavailable` 而 gzserver 在跑 = 容器丢了 GPU（§10.8），先
`docker exec docker-sim-1 nvidia-smi -L`。gzclient 灰屏：`Ctrl+C` 后单独 `ros2 launch gazebo_ros gzclient.launch.py`。

### 4.2 终端 2 — Nav2 + SLAM（nav2_bringup 官方 launch）

```bash
ros2 launch nav2_bringup bringup_launch.py \
    use_sim_time:=true slam:=True autostart:=True use_composition:=False \
    map:=/opt/ros/humble/share/nav2_bringup/maps/turtlebot3_world.yaml \
    params_file:=/opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml
```

`slam:=True` 起 slam_toolbox 提供 /map 与 map→odom；`map:=` 只是 bringup 的必填占位。
R07 最终配置就是 Humble 原版 `nav2_params.yaml`；慢转版换成
`params_file:=$HOME/ws/install/tb3_coordinator/share/tb3_coordinator/config/nav2_params_la_slow.yaml`。
核对：`ros2 action list` 里有 `/navigate_to_pose`；`ros2 topic hz /map`；`ros2 lifecycle list /controller_server`
（或 `ros2 lifecycle get /controller_server` → active）。

### 4.3 终端 3 — 感知链（4 个节点；一条 launch 或逐个 `ros2 run`）

一起起（一个终端）：

```bash
ros2 launch tb3_coordinator perception_la.launch.py                  # 4 个节点
ros2 launch tb3_coordinator perception_la.launch.py detector_config:=$HOME/ws/install/tb3_detector/share/tb3_detector/config/la_detector_search_first.yaml
```

逐个起（4 个终端；launch 文件里每个 Node 就是下面一行，没有别的）：

```bash
# 3a 检测：调 grounding 容器的 POST /detect，发 /detector_node/detections + debug 图
ros2 run tb3_detector la_detector_node --ros-args \
    --params-file $HOME/ws/install/tb3_detector/share/tb3_detector/config/la_detector.yaml -p use_sim_time:=true
# 3b 定位：像素 bearing × LiDAR 距离 → /localizer_node/localized_objects（base_link）
ros2 run tb3_localizer localizer_node --ros-args \
    --params-file $HOME/ws/install/tb3_localizer/share/tb3_localizer/config/localizer.yaml -p use_sim_time:=true
# 3c 短时记忆：观测聚合、1 Hz 重发 → /semantic_memory_node/objects
ros2 run tb3_memory semantic_memory_node --ros-args \
    --params-file $HOME/ws/install/tb3_memory/share/tb3_memory/config/semantic_memory.yaml -p use_sim_time:=true
# 3d 地图记忆：TF 到 map、占据栅格校验、候选→landmark → /semantic_map_memory_node/landmark_objects + RViz 标记
ros2 run tb3_coordinator semantic_map_memory_node --ros-args \
    --params-file $HOME/ws/install/tb3_coordinator/share/tb3_coordinator/config/coordinator.yaml -p use_sim_time:=true
```

核对：`ros2 topic hz /detector_node/detections`（~0.7–0.9 Hz）、`ros2 topic hz /localizer_node/localized_objects`、
`ros2 topic hz /semantic_memory_node/objects`（1 Hz）、`ros2 topic echo /semantic_map_memory_node/landmark_objects --once`。
3a 起来时 grounding 没就绪会刷 `waiting for grounding server`（无害）。

### 4.4 终端 4 — 探索（3 个节点；一条 launch 或逐个 `ros2 run`）

```bash
ros2 launch tb3_frontier_exploration exploration.launch.py                          # 3 个节点，起步扫描 0.4 rad/s
ros2 launch tb3_frontier_exploration exploration.launch.py warmup_angular_speed:=0.3
```

逐个起：

```bash
# 4a 起步旋转扫描（一次性，扫完发 exploration_warmup_complete 后可以 Ctrl+C）
ros2 run tb3_frontier_exploration startup_map_warmup_node.py --ros-args -p use_sim_time:=true -p scan_angular_speed:=0.4
# 4b frontier 检测（/map + /global_costmap/costmap → /frontiers）
ros2 run tb3_frontier_exploration frontier_detection_node --ros-args \
    --params-file $HOME/ws/install/tb3_frontier_exploration/share/tb3_frontier_exploration/config/params.yaml -p use_sim_time:=true
# 4c 目标分配（选 frontier → NavigateToPose；探索完发 /exploration_complete）
ros2 run tb3_frontier_exploration goal_assignment_node --ros-args \
    --params-file $HOME/ws/install/tb3_frontier_exploration/share/tb3_frontier_exploration/config/params.yaml -p use_sim_time:=true
```

核对：`ros2 topic echo /frontiers --once`；4c 日志出现 `[action] Goal accepted (frontier)` = 机器人开始动。

### 4.5 终端 5 — 语义查询 + 目标适配 + 协调器（3 个节点；一条 launch 或逐个 `ros2 run`）

```bash
ros2 launch tb3_coordinator semantic_control.launch.py                               # 3 个节点，LA 词表 registry
ros2 launch tb3_coordinator semantic_control.launch.py semantic_targets_file:=/abs/path/semantic_targets.yaml
```

逐个起：

```bash
# 5a 查询：命令文本 → registry 类名 → 从 landmark 里选目标；LA 词表的 registry 要显式传
ros2 run tb3_query semantic_query_node.py --ros-args \
    --params-file $HOME/ws/install/tb3_query/share/tb3_query/config/semantic_query.yaml -p use_sim_time:=true \
    -p semantic_targets_file:=$HOME/ws/install/tb3_frontier_exploration/share/tb3_frontier_exploration/config/semantic_targets_la.yaml
# 5b 目标适配：landmark 位置 → 接近点位姿 → Nav2 goal
ros2 run tb3_nav_adapter nav_goal_adapter_node --ros-args \
    --params-file $HOME/ws/install/tb3_nav_adapter/share/tb3_nav_adapter/config/nav_goal_adapter.yaml -p use_sim_time:=true
# 5c 协调器状态机：EXPLORING / SEARCHING / SEMANTIC_NAV / TARGET_REACHED / IDLE；订阅 /user_command
ros2 run tb3_coordinator coordinator_node --ros-args \
    --params-file $HOME/ws/install/tb3_coordinator/share/tb3_coordinator/config/coordinator.yaml -p use_sim_time:=true
```

核对：`ros2 topic echo /coordinator_node/status` 出现 `[IDLE] started in IDLE mode — standing still, waiting for /user_command`；
`ros2 node list` 现在应有 ~20 个节点。

**R07 起默认原地等命令**（`coordinator.yaml` `start_mode: "idle"`）：coordinator 启动时 latched 发
`/exploration_enabled=false`，goal_assignment（`start_enabled: false`）与起步扫描节点（`wait_for_exploration_enabled`）
都订阅它（transient_local，所以 ④⑤ 谁先起都一样），第一条 memory 答不上来的命令走搜索路径时才置 true——
那一刻起步扫描先转 ±45°，然后 frontier 探索开始。到达 / NOT FOUND 后回 IDLE 原地等下一条。
旧的"一起来就探索"：`start_mode: "exploring"`；没有 coordinator 单独跑探索：`exploration.launch.py start_enabled:=true`。

### 4.6 终端 6 — RViz（带默认配置文件）

```bash
rviz2 -d $HOME/ws/install/tb3_coordinator/share/tb3_coordinator/rviz/la_demo.rviz --ros-args -p use_sim_time:=true
```

`-d <file.rviz>` 就是"打开时自动加载这套显示配置"，不用逐个 Add topic。两套配置：
- `la_demo.rviz`：录屏布局——地图 + landmark 标记 + 机器人 + Nav2 路径 + 浮动的 detector debug 图，侧栏全隐藏；
- `semantic_nav.rviz`：全套调试面板（Displays / Views 都在）。
自己改好布局后 `File → Save Config As`（Ctrl+S 会直接覆盖打开的那份；install 里的是 symlink，实际写回 `src/tb3_coordinator/rviz/`）。
`-p use_sim_time:=true` 让 RViz 用 /clock，否则 TF 时间对不上会一片报错。

### 4.7 顺序与时序

上面的编号就是顺序：Gazebo（等 `/spawn_entity`、/scan）→ Nav2（等 `/navigate_to_pose`）→ 感知链 →
探索 → 查询/适配/协调 → RViz。六层 = 6 个终端（每层一条 launch）或最多 15 个终端（每节点一个）。
一键版 §4.8 是同一套东西加了 10/15/17/20/23 s 的定时器；手动起的话每层看到核对项再起下一层。
全部起完后回 §5 观察、§6 发命令。

### 4.8 一条命令起全栈（可选，等价于 4.1–4.6）

```bash
ros2 launch tb3_coordinator full_semantic_nav_la.launch.py x_pose:=1.0 y_pose:=-1.0
```

参数：`nav2_params:=`、`warmup_angular_speed:=`、`detector_config:=`、`rviz_config:=`、`world:=`。

## 5. 终端 C — 观察（ros2 自省命令）

这是最通用的一组命令，任何 ROS 2 系统都一样用。另开一个 shell 进同一容器：

```bash
docker exec -it docker-sim-1 bash
source /opt/ros/humble/setup.bash && source ~/ws/install/setup.bash
ros2 node list                                        # 栈起齐了没：la_detector_node / localizer_node / semantic_memory_node /
                                                      #   semantic_map_memory_node / coordinator_node / bt_navigator / controller_server ...
ros2 topic list                                       # 所有 topic；关心的：/user_command /coordinator_node/status
                                                      #   /semantic_map_memory_node/landmark_objects /detector_node/detections
ros2 action list                                      # 出现 /navigate_to_pose 才说明 Nav2 action server 活了（§10.9 竞态）
ros2 param get /semantic_map_memory_node min_distinct_observations   # 核对最终配置（3）
ros2 param get /semantic_memory_node position_smoothing_alpha        # 1.0
ros2 topic echo /coordinator_node/status              # 状态机：EXPLORING → SEMANTIC_QUERYING → SEARCHING/SEMANTIC_NAV → TARGET_REACHED → IDLE
ros2 topic echo /semantic_map_memory_node/landmark_objects --once    # memory 里现在有哪些 landmark（class_id、位置）
ros2 topic hz /detector_node/detections               # 检测有没有在出（~0.7–0.9 Hz）
ros2 topic echo /active_search                        # 搜索中注入的表达式（空 = 没在搜）
```

留证据（可选）：
```bash
ros2 bag record -o ~/ws/reports/R07_probe/bag_takeN /odom /cmd_vel /user_command \
    /coordinator_node/status /semantic_map_memory_node/landmark_objects
```

### 5.1 录屏（宿主机；ffmpeg 原语，窗口手动摆）

launch 默认加载 `src/tb3_coordinator/rviz/la_demo.rviz`（只有地图 + landmark + 浮动的 detector debug
图面板）。把 Gazebo、RViz、debug 图三个窗口手动拖到屏幕左上一块 1920×1080 的区域里平铺
（建议 Gazebo 960×1080 左列，RViz / debug 各 960×540 右列；这台机器的 WM 不让窗口顶到 y<138），
然后：

```bash
ffmpeg -f x11grab -framerate 30 -video_size 1920x1080 -i :1.0+0,138 \
       -c:v h264_nvenc -preset p4 -rc vbr -cq 23 -b:v 0 reports/media/raw_takeN.mp4
```

`-i :1.0+0,138` = 显示 :1、屏幕 0、抓取区域左上角 (0,138)。`Ctrl+C` 停。为什么是小窗平铺而不是整屏：
gzclient 的重绘率与窗口面积成反比（~45 MP/s），整屏 6144×3456 只有 ~4 fps 有效帧；1920×1080 区域
实测 26–29 fps。

**x11grab 抓的是屏幕像素，不是窗口**：录制期间这块屏幕上出现的任何东西都入镜——别切工作区、
别按 Super、关掉通知（R02c、R04、R07 take 2/3 都录进过私人窗口）。录完抽帧看一遍：

```bash
ffmpeg -i reports/media/raw_takeN.mp4 -vf fps=1/15 reports/media/raw_takeN_scan/scan_%03d.jpg
```

## 6. 终端 D — 发命令（ros2 topic pub）

```bash
docker exec -it docker-sim-1 bash
source /opt/ros/humble/setup.bash && source ~/ws/install/setup.bash
ros2 topic pub -1 -w 1 /user_command std_msgs/msg/String "{data: 'find the person wearing a green shirt'}"
```

`-1` 发一条就退出；**`-w 1` 等到有 1 个匹配的订阅者再发**——裸 `--once` 会在 DDS 发现完成前把
第一条丢掉（这个仓库以前用脚本"发布→grep 日志收条→重试"绕它，`-w` 是 ROS 2 自带的正解）。
**不要双发**：coordinator 会把第二条当成抢占。

在终端 C 的 `ros2 topic echo /coordinator_node/status` 看到 `TARGET_REACHED`/`TARGET_FAILED` 后
回到 `IDLE`（到达后 hold，§7.1），再发下一条：

```bash
ros2 topic pub -1 -w 1 /user_command std_msgs/msg/String "{data: 'go to the bookshelf'}"
ros2 topic pub -1 -w 1 /user_command std_msgs/msg/String "{data: 'go to the traffic barrier'}"
ros2 topic pub -1 -w 1 /user_command std_msgs/msg/String "{data: 'find a bicycle'}"
```

每条命令走了哪条路，看 launch 日志（或终端 B 屏幕）：
- `Query OK: bookshelf → bookshelf_0 at (x, y)` 紧跟在 `User command` 后 = **memory 直达**；
- `[search] '...' not in memory — injected into detect vocabulary` = **搜索**（探索/巡逻），之后
  `[search] FOUND ... at (x, y)` 或 `TARGET NOT FOUND`；
- `[new_landmark] <class>_<n> pos=(x, y) after N obs (K distinct)` = memory 新晋升了一个 landmark，
  对照 §9 的真值坐标就知道是不是鬼影；
- `No valid trajectories` / `Goal stalled ... canceling` = Nav2 卡了一下（§10.11）。

## 7. 词表 profile 与切换

| profile | 文件（src/tb3_detector/config/） | 词表 | 用途 |
|---|---|---|---|
| **默认（R05 起）** | `la_detector.yaml` | 5 物理类（person + bookshelf + trash can + traffic cone + concrete barrier） | **建图 + 搜索**：注册表类探索中直接建图、重复命令 memory 直达；颜色表达式与注册表外词走搜索链路 |
| search-first | `la_detector_search_first.yaml` | **空**（coverage 带 5 类 + 4 色衬衫表达式） | R04 演示形态：平时什么都不建图，每条命令都走搜索链路 |
| 四类建图 | `la_detector_four_class.yaml` | person + bookshelf + trash can + traffic cone | R04 前的默认（历史存档，无 barrier） |
| R02c 验收 | `la_detector_person_only.yaml` | 仅 person | R02c 搜索链路验收配置（历史存档） |

两个 profile 的 coverage 里都有 4 条衬衫颜色表达式（green/red/blue/white）——这不是装饰：
* R04 探针实测，只注入 "person wearing a green shirt" + person 时模型会把**错误颜色**的人
  强制匹配成该表达式（score 恒 1.0，6/6 阴性全错）；把其它颜色作为正确备选项一起请求后
  同一组 case 12/12 全对（`reports/R04_probe/`）。颜色搜索能"说不"全靠它们。
* R05 又实测了第二个作用：**颜色撞车保护**。橙色的 traffic cone 标签会强制匹配红衣人、
  绿色的 trash can 标签会强制匹配绿衣人（框直接打在人身上、尺寸正确，几何滤波全部失效）；
  彩衣人自己的表达式 coverage 框与这些错标签框不一致 → veto 掉。没有它们，~10 分钟探索
  就在红衣人身上晋升了 "traffic cone" 假 landmark、绿衣人身上晋升了 "trash can"。

**为什么颜色表达式不直接进词表建图**（R05 步骤 3 试过、两次冒烟失败后回退）：
远处人的表达式框中心列扫到近处物体时按近距落点（R04 §4.4 机理），高度闸（§7.3）能拦；
但"框直接打在目标身上的错标签"（上述颜色撞车 + 蓝表达式在 dedup 里吃掉白衣人的 person
标签）没有几何特征可拦，memory 层会长出改不回来的错色 landmark。证据：
`reports/demo/stack_R05_smoke2_expr_vocab.log`（5 个 landmark 中 4 个是鬼影）。
结果：彩衣人建图归 "person" 类；"find the person wearing a X shirt" 永远走搜索注入
（R04 验收路径），即使已经路过过它。

切换 = 重启栈时给 `detector_config` 参数（先 Ctrl+C 关旧栈，见 §11；新栈 = 全新 memory）：

```bash
ros2 launch tb3_coordinator full_semantic_nav_la.launch.py \
  x_pose:=1.0 y_pose:=1.0 \
  detector_config:=/home/ubuntu/ws/src/tb3_detector/config/la_detector_four_class.yaml \
  2>&1 | tee ~/ws/reports/demo/stack.log
```

就绪行的 `vocabulary=[...]` 是确认 profile 生效的判据（R05 默认 profile 显示 5 个类；
search-first 显示 `vocabulary=[]`）。

### 7.1 到达后保持（hold_after_goal，R04 起默认开）

TARGET_REACHED / TARGET NOT FOUND 后 coordinator 进 **IDLE 原地停**（探索关闭、
cmd_vel 清零），日志判据：

```
[mode] TARGET_REACHED → IDLE
[hold] TARGET_REACHED → IDLE — holding position, exploration stays off
```

发下一条命令即从 IDLE 继续。要回到旧的"到达后 3 s 自动恢复探索"行为：
launch 时给 coordinator 传 `hold_after_goal:=false`（参数在
`src/tb3_coordinator/config/coordinator.yaml`）。

### 7.2 属性修饰查询（R04）

"find the person **wearing a green shirt**" 这类带属性的注册表类命令不再解析成
`person`，而是整句表达式走开放词汇搜索（日志判据：
`Attribute-qualified 'person' → open-vocabulary expression: '...'`）。
纯类名（"go to the person"）与带序号（"go to person 3"）仍走注册表路径。
命令动词表含 find / look for / search for / locate 等。
world 里的三个彩衣 person（绿=中心、蓝=NE、红=SE）由
`scripts/make_person_variants.py` 生成（生成物 gitignore，重建镜像后重跑一次即可）。


**R07 属性挂到身份上（attribute-on-identity）**：以前 `person wearing a green shirt` 的观测在 map memory 里
自己长候选（与 `person_N` 并存、或先在错的人身上凑够 3 帧就晋升——2026-09-17 实测 FOUND 在白衣人上）。现在：
- `semantic_map_memory_node`：表达式观测落在 `attribute_merge_distance_m`（1.5 m）内的基类 landmark（`person`）上时，
  记为该 landmark 的属性证据（按不同检测计数，同晋升防护的 k），不再另起候选；日志 `[attribute] person_2 += '…' (k=2)`，
  gate 行 `attribute`，`landmark_objects` 的 `results[1:]` 带 `(表达式, k)`，RViz 标签第三行 `green_shirt:2`。
  没有基类 landmark 时行为不变（自己长候选，搜索照旧）。
- `semantic_query_node`：表达式没有同名 landmark 时，按属性解析——取 k ≥ `attribute_min_count`（2）里最大的那个
  （并列取最近），日志 `Query OK: … → person_2 … by attribute`。
- `coordinator_node`：搜索期间 landmark 的属性 k ≥ `search_attribute_min_count`（2）也算 FOUND（`via attribute`）。
- 措辞归一化（`query_core.canonicalize_expression`）：`person ware green t shirt` / `person in a red top` /
  `person with blue tshirt` → `person wearing a <colour> shirt`（模型只 ground 这个说法；今天 `ware … t shirt` 138 s 0 框）。
- 未做（备选）：coverage 里的衣色框现在只用于 veto、不发布，所以属性只在搜索期间累积；要"全程从远处累积"得把
  coverage 框也发布成属性观测（R05 见过蓝衣标签框到红衣人，有误标风险）。

**HUD**（`coordinator_node` `hud_*` 参数，`~/hud_markers`，la_demo/semantic_nav.rviz 已加显示）：地图上方
y≈4.3 m 的黑条两行 `cmd:_…` / `state:_…`（空格画成下划线，rviz 空格 bug），状态切换时和每秒各发一次。

### 7.3 R05 的 world 与感知链新增

* **jersey barrier**（西墙，4.07 m 长）：镜像构建期烤入（docker/Dockerfile.sim 末层），
  注册表类 `traffic_barrier`，命令说法 "go to the traffic barrier / jersey barrier /
  barrier"。**detector_label 是 "concrete barrier"**——离线探针 60 帧里
  "traffic barrier" 0 检出、"concrete barrier" 7 帧全对（`reports/R05_probe/`）。
  两个白衣人从 (-2,±2) 角落内移到 (-1.2,±1.8) 给 barrier 让位。
* **高度-距离一致性闸**（localizer，`height_gate_*` 参数）：bbox 高度远小于
  "该类真实高度在射线距离处应有的像素高" → 判定为"远框配了近射线"（误定位）丢弃。
  单侧设计：框偏高 = 近距截断，不拒。R05 冒烟 1/2 里它拦掉了 200+ 次误定位观测
  （日志行 `[height_gate] dropped per class`）。先验表在
  `src/tb3_localizer/config/localizer.yaml`（含 person 家族回退）。
* **debug image 相机帧率化**（detector）：/detect 挪到工作线程，debug 图每帧发布
  （~20 Hz），叠最近一次检测框；框龄 >1.5 s 变灰并标 `(STALE)`。检测节奏不变
  （2.05 Hz / 457 ms 实测同 R04）。
* **搜索巡逻**（coordinator，`search_patrol_*`）：探索干涸后不再 15 s 秒放弃，而是依次
  开到 4 个室内象限点、每点原地慢旋扫视 14 s，预算用完才 NOT FOUND。日志判据
  `[patrol] round n/4 for '<label>' → waypoint (x, y)`。
* **同族标签共存**（detector + memory）：搜索期与搜索词同族（token 子集）的基础词条
  **临时挂起**（掉回 coverage，仍参与遮挡滤波与 veto）；memory 侧同族标签不互为反对
  证据、不互为候选 mutex。没有这两条，词表里的 `person` 会以 503:18 的观测票数压死绿衣
  表达式，且表达式观测会被中心的 person landmark 吞成"反对证据"。
* **搜索晋升门槛分档**：词表**外**的开放词汇表达式用 3（彩衣可见帧只有约 10%），
  词表**内**的类保持常规门槛——B1 实测一刀切成 3 时，书架搜索会在蓝衣人身上以 n=3
  晋升鬼影，机器人开过去"成功"但语义全错。
* **包含关系 veto**（detector，`coverage_containment`）：可发布框有 ≥80% 面积落在标签
  不一致的 coverage 框内 → 判为"局部-整体错标"丢弃（绿衬衫局部被标 trash can 那类）。
* **类级合并半径**（memory，`class_merge_overrides`）：物理尺寸大的类（barrier 2.2 m）
  用自己的合并半径，否则真观测沿 4 m 躯体散布、被切成攒不满的候选。

### 7.4 R06a / R07 的感知时序与解码配置（最终 demo 用的就是这套）

* **localizer 按 stamp 选 scan**（R06a 步骤 1，`scan_stamp_matching`）：射线打进 header stamp
  最近的 /scan（缓存 25 帧），而不是检测结果到达时的最新 scan（比图像晚 ~350 ms，0.4 rad/s
  转弯差 ~10°）。转弯帧落在真实物体上 72% → 82%。日志 `[scan_match]`。
* **memory 层按观测 stamp 查 TF**（R07 步骤 1，`tf_stamped_lookup`/`tf_stamp_timeout` 在
  coordinator.yaml）：semantic_memory_node 每个 Detection3D 的 header.stamp 现在是**该对象最后
  一次观测的图像 stamp**，semantic_map_memory_node 按它查 base_link→map（超时 0.1 s，查不到
  回退 Time(0) 并计数）。之前用 Time(0)：memory 以 1 Hz 重发、对象观测后还"活" 5 s，每次重发
  都用机器人**当前**位姿投影 → 转弯时同一个观测扫出一段弧（bag_turn 回放：垃圾桶 15 次重发
  散布 9.2 m → 1.1 m；全部 170 条 memory 检测落在物体上 13% → 49%，范围闸内 29% → 69%）。
  日志 `[tf_match]`（gate summary 内）。`tf_stamped_lookup: false` 回 R06a 行为。
  `tf_stamp_timeout` 在这个单线程节点里是阻塞上限而非有效等待（等待期间 /tf 进不来），
  最终配置 0.0（R07 冒烟 0.1 时 fallback 0，等待从未起作用）。
* **memory EMA 关掉**（`semantic_memory.yaml` `position_smoothing_alpha: 1.0`，R07）：EMA 在
  base_link 里混不同位姿的观测；bag_turn 回放范围闸内落在真物上 69%（α 0.3）→ 100% / 0.19 m
  （α 1.0，与 localizer 原始输出持平）。
* **晋升防护**（`coordinator.yaml` `min_distinct_observations: 3`，R07 最终）：semantic_memory 以
  1 Hz 重发每个活对象、观测后还活 5 s，TF 修后重发堆到同一点，一次真实检测在 map memory 里算
  3–6 次（垃圾桶 5 次检测 → n=27）。R07 冒烟 1 的鬼影就是 2 帧错标 × 4 次重发 = n=8。现在候选
  除 n≥门槛外还要 ≥3 个不同 (memory 对象 id, 观测 stamp)，搜索表达式同样 3（=3 帧而非 3 次重发）。
  日志：`[held] <class> candidate n=8/8 but k=2/3 distinct ...`（每候选一次）、gate summary 行
  `held_distinct`、候选行 `n=8/8 k=2/3`、`[new_landmark] ... after 8 obs (4 distinct)`。
  **代价（R07 冒烟 2 实测）**：驶过式观测常常只有 2 帧在 2.5 m 闸内，锥桶候选 n=10 k=2 过期 3 次，
  +395 s 才晋升（无防护的冒烟 1 是 +88 s）；书架 +320 s（冒烟 1 +148 s，路径不同不可直接比）。
  搜索类靠巡逻点原地扫（0.5–2 Hz 出框）攒 k，R05 B2 实测 2 s 内 4 帧。
* **解码**（R06a 步骤 3）：greedy（argmax，保留 T/top_p 锐化——直接 T=0 会把 MTP 解码打进
  6 s/帧的重复循环）+ fast 模式 + token 上限 16+24×类别数 + 撞帽自动采样重试；分辨率保持
  640（512 以下 barrier 检出减半）；两阶段 coverage 实现了但默认关。grounding 容器
  compose 里 `GROUNDING_SAMPLING=greedy`、`GENERATION_MODE=fast`，`/health` 可核对。
* **跳帧**（R06a 步骤 4）：FOV 内 3.3 m 无回波、或位移 <0.2 m 且转角 <10° 时不发请求，
  兜底 2 s 一次。`[la_stats]` 报各规则计数。
* **短时跟踪**（R06a 步骤 6）：只填 `Detection2D.id`，memory 不读（课程仓 ByteTrack 教训）。
* **角速度**：R06a 试过全栈 0.3 rad/s，一次冒烟 wedge（676 次 "No valid trajectories"）；
  R07 决定回 R05 的 Humble 原版 Nav2 参数与 0.4 rad/s 起步扫描，0.3 版留作
  `nav2_params_la_slow.yaml` 选项（§4）。

## 8. 演示剧本（建议顺序）

> **R07 最终 demo 剧本 = R05 版剧本，起点固定 (1, -1)，一栈四命令、同一 memory**，
> 最多录 3 次取第一次四条语义全对的（书架/barrier memory 直达或搜索命中且位置对、绿衣
> FOUND、bicycle NOT FOUND 零鬼影；Nav2 超时或 wedge 算失败重录，不剪掉遮掩）。
> 成片 `reports/media/demo_final.mp4`，分段时间表在 `reports/R07_final_demo.md`。
>
> **R05 版剧本（当前默认 profile）= 一栈四命令、同一 memory**（陌生起点，≥1.2 m 净空）：
> ① `find the person wearing a green shirt`（搜索注入→彩衣分辨→REACHED→IDLE；
>    期间探索顺带把注册表类建进 memory）
> ② `go to the bookshelf`（已建图 → memory 直达；未建图 → 继续探索）
> ③ `go to the traffic barrier`（同上；detector_label 是 concrete barrier，§7.3）
> ④ `find a bicycle`（NOT FOUND→IDLE，零幽灵）。
> R04 版剧本（search-first，三条命令各一栈）切
> `detector_config:=.../la_detector_search_first.yaml`。
> R04 实测：绿衣 103–223 s、书架 225 s、bicycle 否定 705 s。
> 下面的四段旧剧本基于四类建图 profile，跑它先切
> `detector_config:=.../la_detector_four_class.yaml`。

### 第一段：四类词表（历史剧本；当时起点 `AUTO` = (-1.5, 0)，R07 起 `AUTO` = (1, -1)）

栈起来后**先让它自主探索 60–90 s**（四类都在词表里，探索路上直接建图；person 晋升门槛 8）。
然后依次发四条命令，每条等 `TARGET_REACHED` 再发下一条：

| 命令 | 预期耗时 | 预期关键日志 |
|---|---|---|
| `go to person` | 10–30 s | `Query OK: person → person_N at (x, y)` → `[mode] SEMANTIC_QUERYING → SEMANTIC_NAV` → `[mode] SEMANTIC_NAV → TARGET_REACHED` |
| `go to the bookshelf` | 15–60 s | 同上，目标 `bookshelf_0`（真值 (0, 2.6) 附近） |
| `go to the trash can` | 15–60 s | 目标 `trash can_0`（真值 (0, -2.3) 附近） |
| `go to the cone` | 15–60 s | 目标 `traffic cone_0`（真值 (2.4, 0) 附近） |

若某类查询时还没晋升（探索没扫到），会看到 `Query MISS with search suggestion` 走搜索流程——
这不是故障，等它 FOUND→REACHED 即可（该词已在词表里，只是把晋升门槛降到 5 加速）。

### 第二段：切验收 profile，演示搜索触发（换起点，如 (1.0, 1.0)）

按 §7 重启栈（全新 memory）。起栈后**尽早发**（查询宜早发，见 §10）：

```bash
ros2 topic pub -1 -w 1 /user_command std_msgs/msg/String "{data: 'go to the trash can'}"
```

预期 25–125 s（R02c 实测三类搜索均值 ~80 s），关键日志序列：

```
Query MISS with search suggestion: inject 'trash can' into the detect vocabulary
[mode] SEMANTIC_QUERYING → SEARCHING
[search] vocabulary injected: 'trash can' (top dedup priority) → categories=['trash can', 'person']
[search] active search class 'trash can' — promotion threshold 5 (instead of 8)
  ...（探索移动中，[la_stats] 的 search='trash can'）...
promote ... trash can: n=5 pos=(...)
[search] FOUND 'trash can' → trash can_0 at (...) n=5 after XXs, X.XXm explored — pausing exploration, re-running query
Query OK: trash_can → trash can_0 at (...)
[mode] SEMANTIC_NAV → TARGET_REACHED
[search] cleared 'trash can' (navigation terminal: TARGET_REACHED) — vocabulary restored; ...
```

### 第三段：否定测试（同一栈继续）

```bash
ros2 topic pub -1 -w 1 /user_command std_msgs/msg/String "{data: 'go to the bicycle'}"
```

bicycle 不在注册表 → 开放词汇路径注入 `'bicycle'` → 探索直到 frontier 耗尽 →
**明确报 NOT FOUND，memory 不长 bicycle 幽灵**。预期 1–6 min（取决于剩余未探索面积；
R02c 全新地图从头探花了 325 s，前面已探过一轮会明显快）。关键日志：

```
Query MISS with search suggestion: inject 'bicycle'
★ EXPLORATION COMPLETE — 15 consecutive empty /frontiers
[search] TARGET NOT FOUND: 'bicycle' — exploration complete, no landmark promoted (searched XXXs, XX.Xm)
[mode] SEARCHING → TARGET_FAILED → (3s) EXPLORING
```

### 第四段：抢占（同一栈继续）

**选离机器人远的已知 landmark**（航程 <10 s 的目标会在你发第二条前就 REACHED，
抢占窗口关闭——R02c 踩过两次）。看 Gazebo 里机器人当前位置，选对角方向的目标：

```bash
ros2 topic pub -1 -w 1 /user_command std_msgs/msg/String "{data: 'go to the trash can'}"
# 看到日志 [mode] ... → SEMANTIC_NAV（goal accepted）后 2–3 s 内：
ros2 topic pub -1 -w 1 /user_command std_msgs/msg/String "{data: 'go to person'}"
```

预期关键日志（第二条入队 → 在途 goal 主动 cancel → 等到终态才派发）：

```
[preempt] old goal terminal (status=5=CANCELED) — dispatching queued 'go to person'
Query OK: person → person_N ...
[mode] SEMANTIC_NAV → TARGET_REACHED
```

全剧本合计约 15–25 min（含一次重启栈）。

## 9. 起点选择规则

**离任何障碍中心 ≥ 1.2 m**（近了 spawn 净空不足，R02b (0.75, 0.75) 实翻过车；
R06a 冒烟起点 (-1.5, 0) 离 barrier 1.05 m，前 100 s goal_assignment 一直 "No low-cost sample"）。
障碍中心（R05 world）：person 在 (0,0)、(2,2)、(2,-2)、**(-1.2, 1.8)、(-1.2, -1.8)**
（两个白衣人 R05 从 (-2,±2) 内移）；bookshelf (0, 2.6)；trash can (0, -2.3)；cone (2.4, 0)；
**jersey barrier x=-2.55、y∈[-2, 2]**（R05 新增）；四面墙在 ±3 m。

验证过的起点（按 R05 world 复核过 1.2 m 规则）：

| 起点 | 最近障碍 | 出处 |
|---|---|---|
| **`(1.0, -1.0)`** | 中心/SE person 1.41 m | **默认预设（`AUTO`，R07 起）**，R05 B1、R07 最终 demo |
| `(0.5, 1.5)` | NW person 1.73 m、中心 1.58 m | R05 A2/A3 |
| `(-1.0, -1.5)` | SW person 0.36 m ✗ | R05 B2（当时能起，但违反规则，不再推荐） |
| `(1.0, 1.0)` | 中心/NE person 1.41 m | R02c run B |
| `(-1.5, 0.0)` | barrier 1.05 m ✗ | 旧默认预设，R02c run C / R06a 冒烟——**R05 加 barrier 后不合规** |
| `(-1.0, -1.0)` | SW person 0.82 m ✗ | R02c run A——**R05 移人后不合规** |

## 10. 已知坑（每条都实测踩过）

1. **`ros2 topic pub --once` 丢首条**：DDS 发现没完成就发了。加 `-w 1`（等到 1 个匹配订阅者再发）即可；仓库里的 `send_command.sh` 是加 `-w` 之前的绕法。
2. **x11grab 抓顶层窗口**：录屏前 `wmctrl -b add,above` 置顶 + 开录后立刻抽帧目检（§5）；
   录屏期间别在同屏操作其他窗口。
3. **gzclient 偶发灰屏**（多见于第二次起栈）：`pkill gzclient` 后重开 gzclient；gzserver 无恙。
4. **查询宜早发**：搜索类查询拖得越晚，误定位观测在目标处堆假 landmark 的时间越长，
   可能进入 relabel 比分僵局（R02c run B bookshelf 638 s 未决）。全新栈起来后想演示
   哪个搜索就尽早发哪条。
5. **grounding 冷启动 ~76 s**：容器 recreate 后首次 /health 要等；不要提前起栈
   （la_detector 会刷 `waiting for grounding server` 节流告警，虽无害但难看）。
6. **抢占演示要选远目标**：航程 6–10 s 的目标抢占窗口太短（§8 第四段）。
7. **杀残留进程别用 `pkill -f "长模式"`**：会先匹配到你自己的 bash 命令行自杀，旧栈没死、
   下次 gzserver 端口占用起不来。先 `ps` 列 PID，再 `kill -9 <纯 PID>`（§11）。
8. **长驻 sim 容器会丢 GPU 访问**（R04 实测）：症状 = rviz2/gzserver/gzclient 全部
   `GLX BadValue (X_GLXCreateContext, value 0x0)`、gzserver 活着但 0 CPU 卡死、
   /spawn_entity 服务不出现。确诊：容器内 `nvidia-smi` 报
   `Failed to initialize NVML: Unknown Error`（宿主 systemd 事件重置了容器的
   cgroup 设备授权）。修复 = `docker compose ... restart sim`（volume 不受影响），
   **然后必须 `restart grounding`**——grounding 共享 sim 的网络命名空间，sim 一重启
   它的 8801 端口就从宿主不可达了。
9. **发命令前等 Nav2 真正激活**（R04 实测一次启动竞态）：只等 coordinator ready 不够，
   goal_assignment 可能一直刷 `Nav2 navigate_to_pose action server not ready`，
   机器人永远不动。判据 = 栈日志出现 `Begin navigating` 或 `[action] Goal accepted`
   再发第一条命令。
10. **GPU 丢失前置检查要能中止**：`docker exec docker-sim-1 nvidia-smi -L | grep -q "GPU 0" || exit 1`。
    R07 用 `nvidia-smi -L | head -1 && ...` 写检查，`head` 吞掉了非零退出码，栈在无 GLX 的容器里
    起了一半（rviz `Unable to create glx context`），白等 5 分钟。起栈后再 grep 一次
    `GLXContext|process has died` 兜底。
11. **frontier goal 贴障碍 → DWB wedge**（R07 take1/2/3、冒烟 2 全部中招）：goal_assignment 选的
    frontier 质心可以离人/锥桶只有 0.4–0.8 m（take1 (2.64,−0.61)、(1.99,−0.26) 贴锥桶；冒烟 2
    (−0.03,−0.44) 贴中心人；take2 (1.25,2.14) 贴 NE 人），DWB 全部轨迹 "Trajectory Hits Obstacle"，
    20 s stalled 取消，多数由下一个 goal 或 fallback 自愈。修法（备选清单，未做）：frontier 候选离
    已知障碍 <0.6 m 不选。
12. **搜索巡逻与 "exploration complete" 的竞态**（R07 take 3，绿衣搜索因此 900 s 超时）：探索经
    stalemate 触发 "EXPLORATION COMPLETE" → 巡逻 round 1 起步 → 1 s 后 goal_assignment 报
    "New frontiers appeared — resuming" → coordinator 收到 complete=False → round 1 扫完后巡逻
    不再推进、探索又被 coordinator 关着 → 机器人原地 578 s 直到下一条命令抢占。目标本身只在
    2.6–3.0 m 出现（12 帧全被 2.5 m 范围闸挡掉）。修法（备选清单，未做）：巡逻一旦开始就把
    complete 标志锁住跑完 4 轮，或 complete 翻回 False 时恢复探索。

## 11. 关栈与释放 GPU 显存

1. 每个节点终端 `Ctrl+C`（用官方 launch 起的话一个 Ctrl+C 全关，等 ~10 s）。
   下次起 gzserver 报端口占用 = 有残留进程，在容器里清（坑 7：先 `ps` 列 PID，再按 PID 杀，
   不要 `pkill -f "长模式"`）：

   ```bash
   ps aux | grep -E "gzserver|gzclient|rviz|ros2|nav2|component|slam" | grep -v grep
   kill -9 <PID ...>
   ```

2. **GPU 显存谁在占、怎么放**：

   ```bash
   nvidia-smi                                                   # 宿主机看总量；Processes 表列出每个进程
   nvidia-smi --query-compute-apps=pid,used_memory,process_name --format=csv
   ```

   两类占用：
   - **grounding 容器里的 `python3 server.py`（LocateAnything-3B，~10 GiB）**：模型常驻显存，只要容器在跑就不释放。
     放掉 = 停容器：`docker compose -f docker/compose.yaml -f docker/compose.local.yaml stop grounding`
     （或 `docker stop docker-grounding-1`）。下次 `... start grounding`，冷启动重新加载权重 ~76 s。
   - **sim 容器里的 gzserver / gzclient / rviz2（各几百 MiB，走 EGL/GLX）**：进程退出就释放，Ctrl+C 即可。

   全停（当天不再跑）：

   ```bash
   cd ~/Desktop/Github_Project/TurtleBot3-semantic-navigation
   docker compose -f docker/compose.yaml -f docker/compose.local.yaml stop     # 只 stop：容器和卷都保留
   nvidia-smi                                                                 # 确认显存回到 ~0
   ```

   **不要 `down -v`**：`docker_hf_cache` 卷里是 LocateAnything-3B 权重；`down`（不带 -v）会删容器但留卷，
   下次要 `up -d` 重建容器（多等一次冷启动），所以日常用 `stop`/`start` 就够。

3. 下次继续：`docker compose ... start` → 回 §3 等 /health。

## 12. 可选封装（仓库脚本各自替代了哪条原语）

主流程不依赖这些；它们是 R02c–R07 验收时为了少敲字写的，留着给回放/判卷用。

| 脚本 | 等价原语 | 多做的事 |
|---|---|---|
| `scripts/send_command.sh "<cmd>" <log>` | `ros2 topic pub -1 -w 1 /user_command std_msgs/msg/String "{data: '<cmd>'}"` | 发完 grep 日志收条、最多重试 3 次（`-w 1` 之前的绕法） |
| `reports/R07_probe/demo_take.sh takeN` | 四条 `ros2 topic pub` + `ros2 topic echo /coordinator_node/status` 等 IDLE + `ros2 bag record` | 写 `takeN_timeline.txt`（epoch 时间戳，对视频用） |
| `scripts/record_demo.sh arrange` | 手动拖窗 | xdotool 把三窗精确平铺成 1920×1080 |
| `scripts/record_demo.sh record out.mp4` | §5.1 那条 `ffmpeg -f x11grab ...` | 3 s 后自动存首帧检查图 |
| `scripts/record_demo.sh check out.mp4` | `ffmpeg -vf fps=1/15` 抽帧 | 加逐秒亮度扫描找入侵窗口 |
| `reports/R07_probe/judge_log.py <log> --mode smoke\|take` | `grep -E "new_landmark\|Query OK\|FOUND\|No valid trajectories\|tf_match\|scan_match" launch.log` | 对照真值坐标算误差、按 R1–R5 规则给 PASS/FAIL |
