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
   [`src/open_space/hybrid_a_star.cc`](src/open_space/hybrid_a_star.cc)：全局路线与泊车连接。
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
    |-- 车道图 A*：自车最近车道 -> 车位入口最近车道
    |-- Hybrid A*：车位入口姿态 -> 车位目标姿态
    +--失败------------------------------------> NO_ROUTE
    v
重复 path_coupling_iterations 次（默认 2 次）
    |-- LocalPlanner：在 S-L 格点图上求无碰撞、曲率可行的横向路径
    +-- SpeedPlanner：在带速度历史的 S-T 格点图上求进度/速度剖面
    v
合成 (path(s), speed(t)) 为 TimedTrajectoryPoint
    |
    +--逐点对动态障碍物做最终碰撞检查--失败--> 紧急制动轨迹 / NO_SAFE_TRAJECTORY
    v
OK + trajectory + diagnostics
```

`Planner::Plan()` 中的关键时序如下：

1. `PlanningFrameAdapter::Adapt()` 依次调用四个适配器，将请求写入 `PlanningFrame`。
2. `GlobalPlanner::Plan()` 生成 `GlobalRoute`：其中既有经过的 `lane_ids`，也有拼接后的
   `reference_line`。
3. 第一次局部规划没有到达时刻信息；`LocalPlanner` 因而按 `t=0` 检查候选路径。
   `SpeedPlanner` 随后给每个离散路径进度生成到达时刻。
4. 顶层把速度剖面回投为每个路径点的 `arrivals`，第二轮局部规划据此在预测时刻检查
   动态障碍物，再重新求速度。这是路径—速度的交替耦合，而非一次联合求解。
5. 通过 `PositionAtS()` 和 `YawAtS()` 将速度点的弧长 `s` 映射回空间路径，形成最终
   `TimedTrajectoryPoint`；最后用 `IsCollisionFree()` 再检验一次。

如果局部路径或速度剖面任一失败，规划器调用 `MakeStopTrajectory()`，沿当前朝向以
`max_deceleration_mps2` 匀减速生成停车轨迹，并在诊断中记录该停车轨迹是否无碰撞。

## 算法原理与代码对应

### 1. 车道拓扑 A*：到达车位入口

[`GlobalPlanner`](src/map/global_planner.cc) 把每条 `Lane` 看作图中的一个节点，
`successor_ids` 看作有向边：

- 起点是离自车位置最近的未关闭车道，终点是离车位 `entry_pose` 最近的未关闭车道。
- 边代价为后继车道中心线长度；启发式为当前车道末端到车位入口的欧氏距离。
- 优先队列按 `f = g + h` 取出节点，记录 `parent` 回溯得到 `lane_ids`。
- 将各车道中心线去重拼接；若最后一点与入口相距超过 0.1 m，额外补上入口点。

这一步只考虑地图拓扑与关闭车道，不考虑动态障碍物；障碍物在之后的局部规划阶段
处理。

### 2. Hybrid A*：从入口连接到车位目标姿态

[`HybridAStar`](src/open_space/hybrid_a_star.cc) 的搜索状态是 `(x, y, yaw)`，兼具连续
姿态和离散查重，因此称为 Hybrid A*。每次扩展使用：

- 固定步长 `0.5 m`；
- 前进、倒车两个方向；
- 三个曲率 `{-0.8, 0, 0.8} * max_curvature_1pm`；
- 栅格键：位置量化到 `0.5 m`，航向量化为每圈 24 格；
- 代价：行驶距离 + 倒车惩罚 `0.2` + 曲率惩罚；
- 启发式：到目标的距离 + `0.5 *` 航向差。

搜索在距离小于 `0.55 m` 且航向差小于 `0.45 rad` 时成功，最多扩展 30,000 个节点。
碰撞检测使用车体和障碍物的有向矩形（长、宽均以各自 `Pose2d` 中心为中心），通过
分离轴定理（SAT）在两矩形的局部轴上进行精确投影判定。`safety_margin_m` 会膨胀自车
矩形的半尺寸，边界接触也视为碰撞。这里仍使用每个障碍物的**第一帧预测**，因此开放空间
连接并未进行完整的动态时域避障。

### 3. 横向路径选择：完整分层 S-L 动态规划

[`LocalPlanner`](src/local/local_planner.cc) 先以 `path_step_m` 重采样 `reference_line`，
将每个弧长采样点作为一层 `S`，并在每层建立 13 个横向状态
`l ∈ {-1.8, -1.5, …, 1.5, 1.8} m`。相邻层之间的状态转移构成完整的分层 S-L 格点图。
为让曲率成为严格可行性条件，DP 状态保存连续两个横向索引 `(l_{i-1}, l_i)`；扩展到
`l_{i+1}` 时即可由连续三个点计算中间点曲率，并记录最佳前驱以供回溯。

- 每个节点沿参考线法线平移到对应 `l`，用该节点的预测到达时刻进行矩形车体碰撞检查。
- 每次转移检查曲率不超过 `max_curvature_1pm`，并对横向偏移、横向斜率和曲率分别计费。
- 在末层选总代价最低的双状态，沿 `parent` 回溯全部横向索引，生成带实际弧长、航向和
  曲率的 `PathPoint`。生成后还会以最终航向再做一次碰撞和曲率校验。

### 4. 纵向速度规划：S-T 动态规划

[`SpeedPlanner`](src/speed/speed_planner.cc) 建立 S-T 格点：横轴是路径离散点的弧长索引
`s`，纵轴是 `0` 到 `horizon_s`、间隔为 `time_step_s` 的时刻 `t`。因为加速度取决于
连续两段速度，状态保存相邻两个进度索引 `(s_{t-1}, s_t)`，而不只是当前 `s_t`。

- 初始状态是 `(t=0, s=0)`。
- 转移只允许 `s_{t+1} >= s_t`，并计算 `v_t = (s_t-s_{t-1})/Δt`、
  `a_t = (v_t-v_{t-1})/Δt`。速度、最大加速度和最大减速度全部是**严格可行性约束**：
  超限状态不会进入格点图，输出阶段不再截断加速度。
- 每个候选 `(t, s)` 使用该时刻障碍物预测和矩形车体检查碰撞。
- 迁移代价为速度跟踪、行驶距离和加速度平方项；终点选择时给予进度奖励。末层状态经由
  `parent` 回溯，因此输出的相邻速度与输出加速度严格一致。

## 模块职责

各规划模块均位于 `src/` 下，头文件与对应的 `.cc` 文件并置，便于一起浏览和维护。模块层级保持为 `common`、`interfaces`、`map`、`open_space`、`local`、`speed` 和 `planning`；例如 `src/interfaces/adapters.h` 与 `src/interfaces/adapters.cc`。工程内包含路径以 `src/` 为根，例如 `#include "planning/planner.h"`。

| 模块 | 关键文件 | 职责 |
| --- | --- | --- |
| `common` | `types.h` | 统一数据模型、配置、状态码和基础几何函数（距离、插值、角度归一化）。 |
| `interfaces` | `adapters.*` | 规划边界：校验外部请求并写入内部 `PlanningFrame`；协议特定转换应放在这里。 |
| `map` | `global_planner.*` | 车道有向图 A*，拼出到车位入口的参考线，并调用开放空间连接器。 |
| `open_space` | `hybrid_a_star.*` | 在可前进、可倒车、受曲率约束的离散状态空间中连接入口与车位目标姿态。 |
| `local` | `local_planner.*` | 在分层 S-L 格点图中规划无碰撞、曲率可行的横向路径，生成带弧长的 `PathPoint`。 |
| `speed` | `speed_planner.*` | 在带速度历史的 S-T 格点图中规划满足严格加减速度限制的进度与速度。 |
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
- 自车位姿、速度、加速度均为有限数且符合车辆纵向限制；每条车道有 ID 和至少两个中心线点。
- 目标车位 ID 非空；地图至少含一条车道。
- 每个障碍物有非空 ID、有限的正长宽、`[0, 1]` 内置信度和非空预测序列；每个预测时刻
  不早于本次请求时间，位姿必须有效。障碍物长宽在该预测序列内固定，预测位姿的航向表示
  其有向矩形朝向。
- 车辆的长、宽、速度/曲率/加减速度限制和安全边距必须有效；`horizon_s >= time_step_s`，
  `time_step_s`、`path_step_m`、`path_coupling_iterations` 必须为正。

成功时，`PlanningResponse::trajectory` 中每个点包含地图系位姿、曲率、速度、加速度和
相对本次请求的时间。可通过 `status` 区分：

| 状态 | 含义 |
| --- | --- |
| `kOk` | 已生成并通过最终矩形车体碰撞检查的时序轨迹。 |
| `kInvalidInput` | 适配器校验失败。 |
| `kNoRoute` | 车位不存在、车道拓扑不可达，或 Hybrid A* 无法接入目标位姿。 |
| `kNoSafeTrajectory` | 局部路径/速度不可行，或最终校验碰撞；响应包含紧急制动降级轨迹。 |
| `kInternalError` | 默认初值；当前顶层流程正常返回时通常不会保留该状态。 |

## 配置、协议与当前实现边界

[`config/default_planner_config.textproto`](config/default_planner_config.textproto) 和
[`proto/planning/v1/planning_config.proto`](proto/planning/v1/planning_config.proto) 定义了
默认配置契约；当前核心库尚未读取 textproto，也没有链接 Protobuf。集成方需要把配置
解析并映射为 `VehicleConfig` / `PlannerConfig`，再传给 `Planner` 构造函数。

阅读和扩展时尤其应留意以下边界：

- `path_step_m` 用于局部 S-L 参考线重采样；`input_max_age_s` 目前仍未使用，输入时效性
  应由集成上游保证。
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
