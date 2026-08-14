# AVP 时空联合动态规划器

这是一个独立的 C++20 自动代客泊车（AVP）规划库。它把外部请求归一化为
`PlanningFrame`，先在车道拓扑中找路线，再连接到车位目标位姿，随后分别规划
横向路径和纵向速度，输出控制器可跟踪的带时间轨迹。

本仓库是一个可读、可编译的 MVP：接口和主干算法已经具备，但部分实现刻意采用
小规模离散近似，不能直接等同于量产规划器。本文以**当前代码实际行为**为准，既
说明它做了什么，也指出阅读时应留意的边界。

## 构建与验证

```bash
cmake -S . -B build -DAVP_BUILD_TESTS=ON
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

`CMakePresets.json` 还提供 `sanitize` 预设。代码按 Google C++ Style 编写，`.clang-format` 与 `.clang-tidy` 已纳入仓库。

### 碰撞几何可视化

`collision_visualizer` 是一个手动运行的调试工具：它将 OBB/SAT 的六个单元测试场景按
每窗口 2×2 子图分页显示。蓝色实线表示自车真实矩形，蓝色虚线表示安全边距膨胀后的
碰撞包络；障碍物为红色时表示碰撞、绿色时表示分离。该工具不属于 CTest，也不会在默认
构建中引入图形依赖。

启用该工具前，需要将 Matplot++ 以 CMake package 形式安装（使
`find_package(Matplot++ CONFIG REQUIRED)` 可用），并安装带桌面终端的 Gnuplot。例如 Debian/Ubuntu：

```bash
sudo apt install gnuplot
```

随后配置、构建并启动窗口：

```bash
cmake --preset visualize
cmake --build --preset visualize --target collision_visualizer
./build/visualize/tools/collision_visualizer
```

若未安装 Matplot++ 或 Gnuplot，只有启用 `AVP_BUILD_VISUALIZERS=ON` 的配置会失败；常规测试构建
不受影响。

### Qt 闭环仿真器

`avp_qt_simulator` 是一个可选的 Qt 5 Widgets 2D 地图查看与闭环仿真工具。它使用完美轨迹
跟踪：每 20 ms 将自车状态直接更新为规划轨迹在下一仿真时刻的插值状态，不模拟真实控制器
或车辆响应滞后。`Run` 开启连续仿真，`Stop` 是唯一会暂停连续仿真的按钮；运行期间每 0.2 秒
滚动调用一次 `Planner::Plan()`。`Reset` 恢复到当前地图文件刚加载后的完整初始状态，包括
场景内容、自车、仿真时间、规划器状态、轨迹和历史记录。程序启动后先显示空画布，使用工具栏
的 `Load map` 加载版本化场景 JSON 后才可运行。车道和车位是只读地图元素；自车起点、目标
车位及动态障碍物关键帧仍可在右侧 `Scenario` 页调整。

规划失败、停车降级和仿真碰撞只在诊断区显示 `WARNING`，不会自动暂停仿真。只要用户没有
点击 `Stop`，仿真时间、动态障碍物和 5 Hz 滚动重规划都会继续推进；没有可执行轨迹时自车
保持当前状态，规划恢复后继续跟踪新轨迹。

当前开发环境需要 Qt 5.15 的 Core 和 Widgets 开发包，例如 Debian/Ubuntu：

```bash
sudo apt install qtbase5-dev
cmake -S . -B build/qt -DAVP_BUILD_VISUALIZERS=ON -DAVP_BUILD_TESTS=ON
cmake --build build/qt --target avp_qt_simulator --parallel
./build/qt/tools/avp_qt_simulator
```

场景使用版本化 JSON 保存。障碍物关键帧会在每个规划周期扩展为完整规划时域的预测，循环
轨迹会重复，非循环轨迹在最后一帧保持，以匹配规划器的零阶保持预测语义。可直接加载
`tools/scenarios/drive_and_park_dynamic.json` 查看包含横穿行人、相邻通道车辆、道路行驶和
开放空间泊车的示例。

右侧 `Plots` 页显示当前规划的 S-L、S-T、S-曲率和解包后的 S-yaw 曲线，以及最近 60 秒
实际自车速度、加速度、yaw 和档位历史。开放空间泊车没有车道 Frenet 参考线，因此泊车时
S-L 图会显示不适用提示，其余三张规划图继续使用当前泊车轨迹更新。图表由 Qt Widgets
直接绘制，不增加 Qt Charts 或 Matplot++ 依赖。

## 从哪里开始读

生产集成时的入口是 [`Planner::Plan`](src/planning/planner.cc)；库本身没有
独立的生产 `main()`。当前唯一可直接运行的示例入口是
[`tests/planning_test.cc`](tests/planning_test.cc) 中的 `main()`：它构造一个最小
地图、调用两次 `Plan`，并验证正常、非法输入、找不到车位和障碍物阻塞四种情况。

建议按以下顺序阅读，能先建立数据模型，再逐步深入算法：

1. [`src/common/types.h`](src/common/types.h)：所有输入、输出、配置和几何基础类型。
2. [`tests/planning_test.cc`](tests/planning_test.cc)：五分钟内了解一次最小调用。
3. [`src/planning/planner.cc`](src/planning/planner.cc)：顶层编排、失败分支和最终安全检查。
4. [`src/interfaces/adapters.cc`](src/interfaces/adapters.cc)：什么输入会被接受或拒绝。
5. [`src/map/global_planner.cc`](src/map/global_planner.cc) 与
   [`src/open_space/hybrid_a_star.cc`](src/open_space/hybrid_a_star.cc)：带档位的开放空间泊车。
6. [`src/local/local_planner.cc`](src/local/local_planner.cc) 与
   [`src/speed/speed_planner.cc`](src/speed/speed_planner.cc)：时空耦合中的路径、速度规划。

## 一次规划的执行流程

```text
PlanningRequest
    |
    v
PlanningFrameAdapter  --非法/缺字段--> INVALID_INPUT
    |  （定位、感知、地图、任务四类校验与归一化）
    v
GlobalPlanner
    +-- 车道图 A*：自车最近车道 -> 车位入口最近车道
    +--失败------------------------------------> NO_ROUTE
    v
按 8 秒最大可达距离 + 制动/车身余量截取滚动局部参考线
    |
重复 path_coupling_iterations 次（默认 2 次）
    |-- LocalPlanner：在 S-L 格点图上求无碰撞、曲率可行的横向路径
    +-- SpeedPlanner：在带速度历史的 S-T 格点图上求进度/速度剖面
    v
合成 (path(s), speed(t)) 为 TimedTrajectoryPoint
    |
    +--逐点对动态障碍物做最终碰撞检查--失败--> 紧急制动轨迹 / NO_SAFE_TRAJECTORY
    v
OK + trajectory + diagnostics

到达车位入口并停车后：Hybrid A* -> 按档位切段 -> 单档位 S-T 速度规划
```

`Planner::Plan()` 中的关键时序如下：

1. `PlanningFrameAdapter::Adapt()` 依次调用四个适配器，将请求写入 `PlanningFrame`。
2. `GlobalPlanner::Plan()` 只生成到车位入口的 `GlobalRoute`；顶层从自车投影处截取有限
   局部参考线，长度为时间域内最大可达距离加制动和车身余量。
3. 第一次局部规划按名义速度估计到达时刻，`SpeedPlanner` 随后给离散路径进度生成更准确
   的到达时刻。
4. 顶层把速度剖面回投为每个路径点的 `arrivals`，第二轮局部规划据此在预测时刻检查
   动态障碍物，再重新求速度。这是路径—速度的交替耦合，而非一次联合求解。
5. 到达入口并停车后切换到开放空间模式；Hybrid A* 保留车身姿态、曲率和前进/倒车方向，
   在 cusp 处分段，换档停车确认后一次只输出一个档位的轨迹。
6. 通过 `PositionAtS()` 和 `YawAtS()` 将速度点映射回空间路径，形成带显式档位的
   `TimedTrajectoryPoint`；最后用 `IsCollisionFree()` 再检验一次。

如果局部路径或速度剖面任一失败，规划器调用 `MakeStopTrajectory()`，沿当前档位方向以
`max_deceleration_mps2` 匀减速生成停车轨迹，并在诊断中记录该停车轨迹是否无碰撞。
该紧急停车轨迹以避碰为优先，不受正常速度规划的舒适 jerk 限制；诊断中会记录
`jerk_constraint=emergency_exempt`。

## 算法原理与代码对应

### 1. 车道拓扑 A*：到达车位入口

[`GlobalPlanner`](src/map/global_planner.cc) 把每条 `Lane` 看作图中的一个节点，
`successor_ids` 看作有向边：

- 自车与车位入口均投影到未关闭车道的中心线**线段**；自车还必须满足配置的距离与航向
  阈值，避免匹配到相邻或反向车道。
- 边代价由实际经过的中心线折线长度组成：起始车道只计投影点后的剩余长度，目标车道只计
  到入口投影点的前缀，中间车道计完整长度。
- 优先队列按 `f = g + h` 取出节点，记录 `parent` 回溯得到 `lane_ids`。
- 参考线依次拼接起始车道尾段、完整中间车道和目标车道前段；它从自车投影点开始并在精确
  的入口位置结束。若起点和入口在同一车道，入口必须位于行驶方向前方。

这一步只考虑地图拓扑与关闭车道，不考虑动态障碍物；障碍物在之后的局部规划阶段
处理。

### 2. Hybrid A*：开放空间单档位分段

[`HybridAStar`](src/open_space/hybrid_a_star.cc) 的搜索状态是 `(x, y, yaw)`，兼具连续
姿态和离散查重，因此称为 Hybrid A*。每次扩展使用：

- 固定步长 `0.5 m`；
- 前进、倒车两个方向；
- 三个曲率 `{-0.8, 0, 0.8} * max_curvature_1pm`；
- 栅格键：位置量化到 `0.5 m`，航向量化为每圈 24 格；
- 代价：行驶距离 + 倒车惩罚 `0.2` + 曲率惩罚；
- 前进/倒车切换额外惩罚 `2.0`，避免离散搜索产生无意义的频繁换档；
- 启发式：到目标的距离 + `0.5 *` 航向差。

搜索在距离小于 `0.55 m` 且航向差小于 `0.45 rad` 时成功，最多扩展 30,000 个节点。
节点同时保存产生它的档位和有符号曲率；回溯结果在方向变化点切成 `ParkingSegment`。
顶层的 `LANE_APPROACH`、`GEAR_SHIFT`、`OPEN_SPACE_PARKING` 状态机只发布当前单一档位段，
默认倒车限速 `1.0 m/s`，车辆停止并等待默认 `1.0 s` 后才能换档。
碰撞检测使用车体和障碍物的有向矩形（长、宽均以各自 `Pose2d` 中心为中心），通过
分离轴定理（SAT）在两矩形的局部轴上进行精确投影判定。`safety_margin_m` 会膨胀自车
矩形的半尺寸，边界接触也视为碰撞。这里仍使用每个障碍物的**第一帧预测**，因此开放空间
连接并未进行完整的动态时域避障。

### 3. 横向路径选择：完整分层 S-L 动态规划

[`LocalPlanner`](src/local/local_planner.cc) 先以 `path_step_m` 重采样滚动截取后的
`reference_line`，
将每个弧长采样点作为一层 `S`，并在每层建立 13 个横向状态
`l ∈ {-1.8, -1.5, …, 1.5, 1.8} m`。相邻层之间的状态转移构成完整的分层 S-L 格点图。
为让曲率成为严格可行性条件，DP 状态保存连续两个横向索引 `(l_{i-1}, l_i)`；扩展到
`l_{i+1}` 时即可由连续三个点计算中间点曲率，并记录最佳前驱以供回溯。

- 每个节点沿参考线法线平移到对应 `l`，用该节点的预测到达时刻进行矩形车体碰撞检查。
- 每次转移检查曲率不超过 `max_curvature_1pm`，并对横向偏移、横向斜率和曲率分别计费。
- 在末层选总代价最低的双状态，沿 `parent` 回溯全部横向索引，生成带实际弧长、航向和
  曲率的 `PathPoint`。生成后还会以最终航向再做一次碰撞和曲率校验。

### 4. 纵向速度规划：S-T 动态规划

[`SpeedPlanner`](src/speed/speed_planner.cc) 在 `0` 到 `horizon_s`、间隔为
`time_step_s` 的时间层上建立稀疏纵向格点。状态直接保存连续的路径进度、速度和加速度
`(s, v, a)`，以 jerk 候选扩展下一层；路径离散点只负责按 `s` 插值位姿并检查碰撞，
不再决定速度分辨率。状态按固定的 `s/v/a` 分辨率合并，并通过确定性的支配和规模剪枝
控制计算量。

- 初始状态是 `(t=0, s=0)`。
- 每次转移使用 `a_{t+1}=a_t+jΔt`、`v_{t+1}=v_t+a_{t+1}Δt` 和
  `s_{t+1}=s_t+v_{t+1}Δt`；首步从自车当前速度和加速度开始。速度、加减速度和正负
  jerk 上下界全部是**严格可行性约束**，输出阶段不做截断。
- 每个候选 `(t, s)` 使用该时刻障碍物预测和矩形车体检查碰撞。
- 普通行驶跟踪传入的速度上限；终点停车时参考速度随剩余距离下降。迁移代价由归一化的
  速度误差、加速度和 jerk 平方项组成并乘以时间步长。要求终点停车时，最终状态必须在
  `0.02 m` 位置容差内同时达到零速度和零加速度，允许提前抵达后保持静止。

## 模块职责

各规划模块均位于 `src/` 下，头文件与对应的 `.cc` 文件并置，便于一起浏览和维护。模块层级保持为 `common`、`interfaces`、`map`、`open_space`、`local`、`speed` 和 `planning`；例如 `src/interfaces/adapters.h` 与 `src/interfaces/adapters.cc`。工程内包含路径以 `src/` 为根，例如 `#include "planning/planner.h"`。

| 模块 | 关键文件 | 职责 |
| --- | --- | --- |
| `common` | `types.h` | 统一数据模型、配置、状态码和基础几何函数（距离、插值、角度归一化）。 |
| `interfaces` | `adapters.*` | 规划边界：校验外部请求并写入内部 `PlanningFrame`；协议特定转换应放在这里。 |
| `map` | `global_planner.*` | 车道有向图 A*，只拼出到车位入口的全局参考线。 |
| `open_space` | `hybrid_a_star.*` | 在可前进、可倒车、受曲率约束的离散状态空间中连接入口与车位目标姿态。 |
| `local` | `local_planner.*` | 在分层 S-L 格点图中规划无碰撞、曲率可行的横向路径，生成带弧长的 `PathPoint`。 |
| `speed` | `speed_planner.*` | 在带加速度历史的 S-T 格点图中规划满足严格加减速度和 jerk 限制的进度与速度。 |
| `planning` | `planner.*` | 编排全部模块，执行两轮路径—速度耦合、轨迹合成、最终校验和紧急停车降级。 |
| `proto` | `proto/planning/v1/*.proto` | 集成边界的 Protobuf 契约；本仓库不提交生成的绑定代码。 |
| `config` | `default_planner_config.textproto` | 示例默认参数，字段与 `VehicleConfig`、`PlannerConfig` 对应。 |

## 输入、输出与关键约束

核心调用形态如下。`Planner` 构造时可传入车辆和规划参数；若省略则使用
[`types.h`](src/common/types.h) 中的默认值。

```cpp
avp::Planner planner(vehicle_config, planner_config);
avp::PlanningResponse response = planner.Plan(request);
```

`PlanningRequest` 必须满足以下约束，否则返回 `kInvalidInput`：

- `header.frame_id == "map"`，且 `timestamp_ns` 和 `sequence_id` 都非零。
- 自车位姿、速度、加速度均为有限数且符合车辆纵向限制；每条车道有唯一 ID、至少两个有限
  中心线点和非零长度线段，且每个后继 ID 均存在。
- 目标车位 ID 非空；地图至少含一条车道；停车位 ID 唯一且入口、目标位姿均有限。
- 每个障碍物有非空 ID、有限的正长宽、`[0, 1]` 内置信度和非空预测序列；每个预测时刻
  不早于本次请求时间，位姿必须有效。障碍物长宽在该预测序列内固定，预测位姿的航向表示
  其有向矩形朝向。
- 车辆的长、宽、速度/曲率/加减速度/jerk 限制和安全边距必须有效；jerk 代价权重必须为
  有限非负数；`horizon_s >= time_step_s`，
  `time_step_s`、`path_step_m`、`path_coupling_iterations`、车道匹配距离和航向阈值必须为正，
  且航向阈值不超过 π；倒车限速不得超过车辆限速，换档停止阈值和驻留时间不得为负。

成功时，`PlanningResponse::trajectory` 中每个点包含地图系位姿、曲率、非负速度大小、
加速度、相对时间和显式 `DRIVE`/`REVERSE` 档位。可通过 `status` 区分：

| 状态 | 含义 |
| --- | --- |
| `kOk` | 已生成并通过最终矩形车体碰撞检查的时序轨迹。 |
| `kInvalidInput` | 适配器校验失败。 |
| `kNoRoute` | 车位不存在或到车位入口的车道拓扑不可达。 |
| `kNoSafeTrajectory` | 局部路径/速度不可行，或最终校验碰撞；响应包含紧急制动降级轨迹。 |
| `kInternalError` | 默认初值；当前顶层流程正常返回时通常不会保留该状态。 |

## 配置、协议与当前实现边界

[`config/default_planner_config.textproto`](config/default_planner_config.textproto) 和
[`proto/planning/v1/planning_config.proto`](proto/planning/v1/planning_config.proto) 定义了
默认配置契约；当前核心库尚未读取 textproto，也没有链接 Protobuf。集成方需要把配置
解析并映射为 `VehicleConfig` / `PlannerConfig`，再传给 `Planner` 构造函数。

阅读和扩展时尤其应留意以下边界：

- `path_step_m` 用于局部 S-L 参考线重采样；`input_max_age_s` 目前仍未使用，输入时效性
  应由集成上游保证。`max_lane_match_distance_m` 与
  `max_lane_heading_difference_rad` 默认分别为 `2.0 m` 与 `60°`，用于约束自车挂接道路车道。
- 纵向 jerk 默认边界为 `[-4.0, 2.0] m/s³`，平方代价权重默认为 `1.0`。车道接近阶段
  以车辆最大速度为目标；开放空间泊车通用限速默认为 `1.0 m/s`，倒车还会受到
  `max_parking_speed_mps` 和 `max_reverse_speed_mps` 中更严格者的约束。这些只是初始工程
  参数，实车集成应根据驱动/制动响应、控制周期和舒适性数据重新标定。
- 局部路径的第一个点固定为自车当前位姿；若无法以曲率约束连接到参考线，规划会走现有的
  安全停车降级，而不会输出位置断裂的轨迹。
- 预测查询采用“取不晚于目标时刻的最后一个离散预测点”的零阶保持，不做插值；早于
  第一预测点时则使用第一点。
- 所有碰撞模型均采用自车与障碍物有向矩形的 SAT 静态相交检测；自车半尺寸按
  `safety_margin_m` 保守膨胀，未覆盖障碍物多边形轮廓、车辆扫掠体或时变尺寸。
- S-L 与 S-T 都是离散格点 DP；前者的横向范围和分辨率当前固定为 `±1.8 m`、`0.3 m`，
  后者只允许沿参考路径前进，均尚未引入地图车道边界或倒车速度状态。
- 安全降级只是沿当前航向直线制动，并不保证一定无碰撞；结果通过
  `stop_collision_free=true/false` 写入 `diagnostics`，上层必须据此决定进一步动作。

这些限制不是接口限制：例如可保留 `PlanningFrame` 和 `PlanningResponse` 不变，逐步替换
障碍物插值、障碍物多边形、扫掠体、连续优化或更精确的开放空间规划器。

## 集成边界

- 外部消息契约位于 [`proto/planning/v1/planning.proto`](proto/planning/v1/planning.proto)。
  `Obstacle.radius_m` 已保留字段号并废弃；集成方必须改为提供 `length_m` 与 `width_m`，
  否则输入会因缺少有效尺寸而返回 `kInvalidInput`。
  当前核心零第三方依赖；安装 Protobuf 后，应由集成项目生成绑定，并在
  `LocalizationAdapter`、`PerceptionAdapter`、`MapAdapter`、`TaskAdapter` 中或其上游完成
  ROS 2、Apollo、车辆协议到内部类型的映射。
- 请求使用 `map` 坐标系和 SI 单位。当前代码只检查 `timestamp_ns` 与 `sequence_id` 非零，
  **不会**保存上一请求并验证两者递增；若集成需要时序防护，应在上游或适配器扩展中加入。
- 动态障碍物需提供从请求时刻开始的预测点；不完整输入返回 `kInvalidInput`。状态码的
  字符串形式可由 `ToString(PlanningStatus)` 获取。
