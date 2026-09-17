# 快速上手（只讲怎么跑）

细节、坑、参数含义都在 [la_runbook.md](la_runbook.md)；这页只有命令。

## 0. 起容器（宿主机，每天一次）

```bash
cd ~/Desktop/Github_Project/TurtleBot3-semantic-navigation
xhost +local:docker
docker compose -f docker/compose.yaml -f docker/compose.local.yaml up -d
docker exec docker-sim-1 nvidia-smi -L                # 必须打印 GPU 0；否则 restart sim，再 restart grounding
curl http://127.0.0.1:8801/health                     # 等到 "device": "cuda"（冷启动 ~76 s）
```

## 1. 每个终端开头（进容器 + source，逐行敲）

```bash
docker exec -it docker-sim-1 bash

```

```bash
source /opt/ros/humble/setup.bash
source ~/ws/install/setup.bash

```

## 2. 六层，按顺序，每层一个终端

```bash
# ① Gazebo + 机器人
ros2 launch tb3_frontier_exploration gazebo_sim.launch.py                  # 默认 person world，起点 (1,-1)
#    换世界/起点：world:=warehouse_models x_pose:=-1.2 y_pose:=-1.2  ；不开 GUI：gui:=false
#    就绪：ros2 service list | grep spawn  →  /spawn_entity

# ② Nav2 + SLAM（官方 launch）
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=true slam:=True autostart:=True use_composition:=False \
    map:=/opt/ros/humble/share/nav2_bringup/maps/turtlebot3_world.yaml \
    params_file:=/opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml
#    就绪：ros2 action list  →  /navigate_to_pose

# ③ 感知链（4 节点）
ros2 launch tb3_coordinator perception_la.launch.py
#    就绪：ros2 topic hz /detector_node/detections   （~0.8 Hz）

# ④ 探索（3 节点）——起来后机器人不动，等 ⑤ 的第一条搜索命令才开始探索
ros2 launch tb3_frontier_exploration exploration.launch.py
#    就绪：日志 [exploration] DISABLED via /exploration_enabled（⑤ 起来后出现）

# ⑤ 查询 + 适配 + 协调（3 节点）——默认 start_mode: idle，原地等命令
ros2 launch tb3_coordinator semantic_control.launch.py
#    就绪：ros2 topic echo /coordinator_node/status  →  [IDLE] started in IDLE mode — standing still, waiting for /user_command

# ⑥ RViz（带默认配置）
rviz2 -d $HOME/ws/install/tb3_coordinator/share/tb3_coordinator/rviz/la_demo.rviz --ros-args -p use_sim_time:=true
```

## 3. 发命令（第 7 个终端，同样先进容器 + source）

```bash
ros2 topic pub -1 -w 1 /user_command std_msgs/msg/String "{data: 'find the person wearing a green shirt'}"
```

在另一个终端 `ros2 topic echo /coordinator_node/status` 看到 `TARGET_REACHED` → `IDLE` 后再发下一条：

```bash
ros2 topic pub -1 -w 1 /user_command std_msgs/msg/String "{data: 'go to the bookshelf'}"
ros2 topic pub -1 -w 1 /user_command std_msgs/msg/String "{data: 'go to the traffic barrier'}"
ros2 topic pub -1 -w 1 /user_command std_msgs/msg/String "{data: 'find a bicycle'}"      # 会 NOT FOUND
```

`-1` 发一条退出，`-w 1` 等订阅者出现再发（不加会丢首条）。不要连发两次（第二条算抢占）。
memory 里已有 → 日志 `Query OK: … → bookshelf_0 at (x, y)` 直达；没有 → `[search] … injected, exploration enabled`：
这时探索才开始（起步先 ±45° 扫一下，然后建图找目标），到达/找不到后回 IDLE 原地等下一条。
想一起来就自由探索（旧行为）：⑤ 加 `start_mode:=exploring`？——不是 launch 参数，改 `coordinator.yaml` 的
`start_mode: "exploring"`；只跑 ④ 不跑 ⑤ 时：`exploration.launch.py start_enabled:=true`。

## 4. 单节点调试（只在需要时）

③④⑤ 每个 launch 里的节点 = 一行 `ros2 run`，格式都是
`ros2 run <包> <可执行> --ros-args --params-file <yaml> -p use_sim_time:=true`。把 launch 换成其中几行即可：

```bash
I=$HOME/ws/install
# ③ 感知链
ros2 run tb3_detector    la_detector_node         --ros-args --params-file $I/tb3_detector/share/tb3_detector/config/la_detector.yaml       -p use_sim_time:=true
ros2 run tb3_localizer   localizer_node           --ros-args --params-file $I/tb3_localizer/share/tb3_localizer/config/localizer.yaml        -p use_sim_time:=true
ros2 run tb3_memory      semantic_memory_node     --ros-args --params-file $I/tb3_memory/share/tb3_memory/config/semantic_memory.yaml     -p use_sim_time:=true
ros2 run tb3_coordinator semantic_map_memory_node --ros-args --params-file $I/tb3_coordinator/share/tb3_coordinator/config/coordinator.yaml -p use_sim_time:=true
# ④ 探索
ros2 run tb3_frontier_exploration startup_map_warmup_node.py --ros-args -p use_sim_time:=true -p scan_angular_speed:=0.4
ros2 run tb3_frontier_exploration frontier_detection_node    --ros-args --params-file $I/tb3_frontier_exploration/share/tb3_frontier_exploration/config/params.yaml -p use_sim_time:=true
ros2 run tb3_frontier_exploration goal_assignment_node       --ros-args --params-file $I/tb3_frontier_exploration/share/tb3_frontier_exploration/config/params.yaml -p use_sim_time:=true
# ⑤ 查询 / 适配 / 协调
ros2 run tb3_query       semantic_query_node.py --ros-args --params-file $I/tb3_query/share/tb3_query/config/semantic_query.yaml -p use_sim_time:=true \
                                                  -p semantic_targets_file:=$I/tb3_frontier_exploration/share/tb3_frontier_exploration/config/semantic_targets_la.yaml
ros2 run tb3_nav_adapter nav_goal_adapter_node  --ros-args --params-file $I/tb3_nav_adapter/share/tb3_nav_adapter/config/nav_goal_adapter.yaml -p use_sim_time:=true
ros2 run tb3_coordinator coordinator_node       --ros-args --params-file $I/tb3_coordinator/share/tb3_coordinator/config/coordinator.yaml      -p use_sim_time:=true
```

常用查看：`ros2 node list`、`ros2 topic list`、`ros2 topic hz <topic>`、`ros2 topic echo <topic> --once`、
`ros2 param get <node> <param>`、`ros2 launch <pkg> <file> --show-args`。

## 5. 关

```bash
# 每个终端 Ctrl+C；然后宿主机：
docker compose -f docker/compose.yaml -f docker/compose.local.yaml stop     # 释放 ~10 GB 显存；只 stop，不 down
nvidia-smi                                                                 # python3 那行消失
```
