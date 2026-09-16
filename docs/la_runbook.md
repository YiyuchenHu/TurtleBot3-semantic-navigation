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

先在 sim 容器里 `docker exec docker-sim-1 nvidia-smi -L`：报 `NVML: Unknown Error`
就是丢了 GPU（§10 坑 8），按那条先 restart sim 再 restart grounding。

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

进入容器后（起点坐标按 §9 规则选，`AUTO` = 预设 **(1, -1)**，R07 起；R05 加 barrier 后
旧预设 (-1.5, 0) 离它只有 1.05 m，已不合规）：

```bash
source /opt/ros/humble/setup.bash && source ~/ws/install/setup.bash
ros2 launch tb3_coordinator full_semantic_nav_la.launch.py \
  x_pose:=1.0 y_pose:=-1.0 2>&1 | tee ~/ws/reports/demo/stack.log
```

R07 最终配置 = 这条命令的默认值：Nav2 用 Humble 原版 `nav2_bringup/params/nav2_params.yaml`
（R05 配置），起步旋转扫描 0.4 rad/s，memory 层按观测 stamp 查 TF（`tf_stamped_lookup: true`、
`tf_stamp_timeout: 0.0`）、semantic_memory `position_smoothing_alpha: 1.0`、map memory 晋升防护
`min_distinct_observations: 3`（§7.4）。起栈后可用 `ros2 param get` 核对这四项；
`SemanticMapMemoryNode ready ... min_obs=8 min_distinct=3` 一行也能看到。
可选慢转版（R06a 步骤 7 的 0.3 rad/s 全套限速，R07 未采用为默认）：

```bash
ros2 launch tb3_coordinator full_semantic_nav_la.launch.py \
  nav2_params:=/home/ubuntu/ws/install/tb3_coordinator/share/tb3_coordinator/config/nav2_params_la_slow.yaml \
  warmup_angular_speed:=0.3 2>&1 | tee ~/ws/reports/demo/stack.log
```

启动就绪判据（launch 后 ~25–40 s 内依次出现）：

```
[la_detector_node]: la_detector_node ready. server=http://127.0.0.1:8801 vocabulary=['person', 'bookshelf', 'trash can', 'traffic cone', 'concrete barrier'] ...
[bt_navigator]: Begin navigating ...      ← 或探索开始、机器人在 Gazebo 里动起来
[la_detector_node]: [la_stats] frames=... requests=... errors=0 ...
```

`[la_stats]` 行每 10 s 一条，`errors=0` 是 grounding 链路健康的持续证据。
另外两条健康判据（R06a/R07）：localizer 的 `[scan_match] stamp-matched N, fallback-to-latest 0`
和 semantic_map_memory_node 的 gate summary 里 `[tf_match] stamped N, fallback-to-latest 0`——
两个 fallback 计数都应为 0 或接近 0（非 0 = 时序退化，见 §7.4）。

**冒烟 / take 判卷一律跑** `python3 reports/R07_probe/judge_log.py reports/demo/stack.log`：
打印 wedge 计数（R1）、两个 fallback（R2）、每个 landmark 事件（new_landmark / relabel / held）
落在哪个真物上或 GHOST、结束 landmark 是否全在真物上（R3）、命令四类任何时刻有无鬼影（R4，
其它类被 relabel 自愈的列为 self-healed 不算失败）。历史校准（"No valid trajectories" 行数 /
Goal stalled 次数）：R05 B1 0/3、R05 A2 8/7、R07 冒烟 1 0/5、R07 冒烟 2 38/11（70 s 卡在中心人物旁，
fallback 自愈）、R07 take1/2/3 142/·、16/·、32/·、R06a 0.3 rad/s 868/6。
**R07 定稿判据**：冒烟 R1 = 无 wedge，或 wedge 全部由 fallback 自愈且探索完成（`--mode smoke`）；
take R1 = 任何 wedge / Nav2 超时即作废（`--mode take`，还打印每条命令 memory 直达/搜索、终态、
位置误差、耗时，末行 `TAKE PASS/FAIL`）。R07 实测 (1,−1) 起点 5 次起栈 4 次有 wedge（frontier goal
选在离人/锥桶 0.4–0.8 m 处），严格 take 规则在 frontier 选点净空修好之前基本过不了（§10.11）。

## 5. 录屏准备（发第一条命令前做完）

x11grab 抓的是**区域内最顶层的像素**——目标窗口没铺满录制区域、或被别的窗口/弹窗
盖住，就会把你的桌面录进去（R02c 删过全部 RViz 素材；R04 又因 Gazebo 没铺满左半屏
报废过一条，成片前必须逐段抽帧扫描）。录屏期间不要在同一屏幕上干别的，**先关掉
通知弹窗**（微信等常驻通知会飘在最上层）。

### 5.1 RViz 录屏布局（R04 起，R05 改紧凑版）

launch 默认加载 `src/tb3_coordinator/rviz/la_demo.rviz`：只有地图视图
（SLAM 地图 + landmark MarkerArray + 机器人 + Nav2 路径）和一块**浮动的**
la_detector debug image 大面板，Displays/Views/Tool Properties/Time 全部隐藏。
要全套调试面板时传 `rviz_config:=<绝对路径>/semantic_nav.rviz`。

**R07 又中招一次**：take 2 录到 298–343 s 时一个私人笔记窗口盖住了 Gazebo/RViz 列——正好是
书架、barrier 两条 memory 直达命令；take 3 录到 GNOME 概览与 Files 窗。录屏期间这块屏幕
**什么都不能碰**（切工作区、Super 键、通知都会入镜）。`record_demo.sh check` 现在除 15 s 抽帧外
还自动做逐秒亮度扫描（Gazebo 列 YAVG 偏离中位 >12 的秒段 = 疑似入侵），15 s 网格漏过的 3 s 段
它能抓到；用 `ffmpeg select='not(between(t,a,b)+...)'` 切掉后再扫一遍到 "none"。

### 5.2 摆窗与录制（R05 管线：scripts/record_demo.sh）

**R04 管线（整屏抓取 + libx264）产出的是 ~4 fps 的幻灯片**——R05 逐帧 YDIF 实测
raw_R04_G3 有效帧率 3.9 fps。凶手不是编码器而是 gzclient 本身：它的重绘率与窗口
面积成反比（~45 MP/s 吞吐上限），3072×3406 的大窗只能画 4-5 fps；同管线抓 400×400
的 glxgears 能到 55/60 fps。修法 = 三窗口缩小平铺一块**原生 1920×1080 区域**
（零缩放）+ GPU 编码（h264_nvenc），实测行驶中有效帧率 26-29 fps。

```bash
# 起栈后（Nav2 激活、三窗口都已出现）：
scripts/record_demo.sh arrange          # 摆窗：Gazebo 960x1080 左列，RViz/debug 各 960x540 右列
scripts/record_demo.sh record reports/media/raw_demo.mp4 900 &   # 30fps NVENC，开录 3s 后自动存首帧检查图
# 结束：Ctrl+C / kill -INT 录制进程；然后全扫：
scripts/record_demo.sh check reports/media/raw_demo.mp4          # 15s 间隔抽帧到 *_scan/，逐帧目检
```

摆窗细节都在脚本里：窗口识别要挑**客户窗**（RViz 会有一个带 "(on <容器id>)" 的
WM 框架窗同名混进搜索结果，对它 move/resize 全是无效操作）；xdotool 的
getwindowgeometry 会叠加虚假的边框偏移，脚本以宿主机 `xwininfo` 的绝对坐标为真值
迭代收敛；WM 拒绝把带装饰的窗放到 y<138，所以布局锚定在 (0,138)。
录制区域 = 三窗口平铺的精确矩形，区域外的桌面永远不入镜。

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
