# LocateAnything 模式运行手册（la_runbook）

手动开终端跑通 LocateAnything 语义导航栈并录屏的操作手册。每个终端一节，命令逐条可直接粘贴。数据（预期耗时、预期日志行）全部来自 R02b/R02c 验收实测。

宿主机前提：NVIDIA 驱动 + NVIDIA Container Toolkit + X11 桌面；仓库位于
`~/Desktop/Github_Project/TurtleBot3-semantic-navigation`（下文简写 `$REPO`）。

---

## 1. 总览与架构

一次完整演示 = 3 个终端（A 容器 / B ROS 栈 / C 发命令）+ 可选录屏终端 D。

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
la_detector 的 detect 词表（最高去重优先级）并开探索；semantic_memory 对搜索类把晋升
门槛从 8 降到 5；landmark 出现后 coordinator 暂停探索、重发原命令走正常导航链。

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

## 3. 终端 A — 起容器，等 /health

```bash
cd ~/Desktop/Github_Project/TurtleBot3-semantic-navigation
xhost +local:docker
docker compose -f docker/compose.yaml -f docker/compose.local.yaml up -d
```

等 grounding 就绪（**冷启动约 76 s**——容器 recreate 或权重页缓存冷时；热启动约 10 s）：

```bash
until curl -sf http://127.0.0.1:8801/health; do echo waiting...; sleep 5; done; echo
```

预期输出（`device` 必须是 `cuda`）：

```
{"status": "ok", "backend": "locate_anything", "device": "cuda", "model": "nvidia/LocateAnything-3B", "dtype": "bfloat16"}
```

## 4. 终端 B — 起 ROS 栈

日志固定写到 `reports/demo/stack.log`（宿主可见，终端 C 的 send_command.sh 靠它验证收条）。
**每次重启栈会覆盖该文件**；想留档先 `mv reports/demo/stack.log reports/demo/stack_runN.log`。

```bash
cd ~/Desktop/Github_Project/TurtleBot3-semantic-navigation
mkdir -p reports/demo
docker compose -f docker/compose.yaml -f docker/compose.local.yaml exec sim bash
```

进入容器后（起点坐标按 §9 规则选，`AUTO` = 预设 (-1.5, 0)）：

```bash
source /opt/ros/humble/setup.bash && source ~/ws/install/setup.bash
ros2 launch tb3_coordinator full_semantic_nav_la.launch.py \
  x_pose:=-1.5 y_pose:=0.0 2>&1 | tee ~/ws/reports/demo/stack.log
```

启动就绪判据（launch 后 ~25–40 s 内依次出现）：

```
[la_detector_node]: la_detector_node ready. server=http://127.0.0.1:8801 vocabulary=['person', 'bookshelf', 'trash can', 'traffic cone'] ...
[bt_navigator]: Begin navigating ...      ← 或探索开始、机器人在 Gazebo 里动起来
[la_detector_node]: [la_stats] frames=... requests=... errors=0 ...
```

`[la_stats]` 行每 10 s 一条，`errors=0` 是 grounding 链路健康的持续证据。

## 5. 录屏准备（发第一条命令前做完）

x11grab 抓的是**区域内最顶层窗口**——RViz/Gazebo 被别的窗口盖住就会录到你的桌面
（R02c 实翻过车，RViz 路素材因此全删）。录屏期间不要在同一屏幕上干别的。

```bash
wmctrl -l                                   # 找到 Gazebo / RViz 窗口
wmctrl -r "RViz" -b add,above               # 置顶
wmctrl -r "Gazebo" -b add,above
```

录制（终端 D，按 `wmctrl -lG` 得到的窗口几何改 X,Y,WxH）：

```bash
ffmpeg -f x11grab -framerate 30 -video_size 1600x960 -i :0.0+0,0 \
  ~/Desktop/demo_raw.mp4
```

**开始录制后立刻抽一帧目检**（另开终端）：

```bash
ffmpeg -y -sseof -1 -i ~/Desktop/demo_raw.mp4 -frames:v 1 /tmp/check.jpg && xdg-open /tmp/check.jpg
```

gzclient 偶发全屏灰屏（第二次起栈时出现过）：`pkill gzclient` 后在容器里
`gzclient &` 重开即可，世界与 gzserver 不受影响。

## 6. 终端 C — 发命令（一律走 send_command.sh）

`ros2 topic pub --once` 丢首条是常态；脚本发布后 grep 栈日志收条，未收到自动重试
（最多 3 次）。**不要手动双发**——coordinator 的抢占逻辑会把它当成两条指令。

```bash
cd ~/Desktop/Github_Project/TurtleBot3-semantic-navigation
./scripts/send_command.sh "go to person" reports/demo/stack.log
```

预期输出 `RECEIVED (attempt 1): go to person`；栈日志同时出现
`[coordinator_node]: User command: 'go to person'`。

## 7. 词表 profile 与切换

| profile | 文件（src/tb3_detector/config/） | 词表 | 用途 |
|---|---|---|---|
| **默认** | `la_detector.yaml` | person + bookshelf + trash can + traffic cone | 日常运行：四类全部在探索中建图 |
| **验收（搜索演示）** | `la_detector_person_only.yaml` | 仅 person | 三个物体变成词表外目标，查询必然触发搜索链路（R02c 验收配置） |

切换 = 重启栈时给 `detector_config` 参数（先 Ctrl+C 关旧栈，见 §11；新栈 = 全新 memory）：

```bash
ros2 launch tb3_coordinator full_semantic_nav_la.launch.py \
  x_pose:=1.0 y_pose:=1.0 \
  detector_config:=/home/ubuntu/ws/src/tb3_detector/config/la_detector_person_only.yaml \
  2>&1 | tee ~/ws/reports/demo/stack.log
```

就绪行变为 `vocabulary=['person']` ——这是确认 profile 生效的判据。

## 8. 演示剧本（建议顺序）

### 第一段：默认四类词表（起点 `AUTO` 即 (-1.5, 0)）

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
./scripts/send_command.sh "go to the trash can" reports/demo/stack.log
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
./scripts/send_command.sh "go to the bicycle" reports/demo/stack.log
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
./scripts/send_command.sh "go to the trash can" reports/demo/stack.log
# 看到日志 [mode] ... → SEMANTIC_NAV（goal accepted）后 2–3 s 内：
./scripts/send_command.sh "go to person" reports/demo/stack.log
```

预期关键日志（第二条入队 → 在途 goal 主动 cancel → 等到终态才派发）：

```
[preempt] old goal terminal (status=5=CANCELED) — dispatching queued 'go to person'
Query OK: person → person_N ...
[mode] SEMANTIC_NAV → TARGET_REACHED
```

全剧本合计约 15–25 min（含一次重启栈）。

## 9. 起点选择规则

**离任何障碍中心 ≥ 1.2 m**（近了 spawn 净空不足，R02b (0.75, 0.75) 实翻过车）。
障碍中心：5 个 person 在 (0,0)、(±2,±2)；bookshelf (0, 2.6)；trash can (0, -2.3)；
cone (2.4, 0)；四面墙在 ±3 m。

验证过的起点（R02b/R02c 各 run 实际用过）：

| 起点 | 出处 |
|---|---|
| `(-1.5, 0.0)` | 默认预设（`AUTO`），R02c run C |
| `(-1.0, -1.0)` | R02c run A |
| `(1.0, 1.0)` | R02c run B |

## 10. 已知坑（每条都实测踩过）

1. **`ros2 topic pub --once` 丢首条**：一律走 `scripts/send_command.sh`（发布→收条→重试）。
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

## 11. 关栈

1. 终端 B：`Ctrl+C`，等 launch 各进程退出（~10 s）。若下次起栈报 gzserver 端口占用，
   在容器里清残留（注意坑 7，用 PID 不用 pkill -f）：

   ```bash
   ps aux | grep -E "gzserver|gzclient|rviz|ros2|nav2|component" | grep -v grep
   kill -9 <上面列出的 PID...>
   ```

2. 录完当天不再跑：停容器释放显存（grounding 驻留 ~10 GiB VRAM）。
   **只 stop，不 down、不删卷**：

   ```bash
   cd ~/Desktop/Github_Project/TurtleBot3-semantic-navigation
   docker compose -f docker/compose.yaml -f docker/compose.local.yaml stop
   ```

3. 下次继续：`docker compose ... start` → 回 §3 等 /health。
