# 仓库指南

## 项目结构与模块组织

`src/` 按规划阶段划分为 `common`、`interfaces`、`map`、`open_space`、
`local`、`speed` 和 `planning`。公开头文件与其 `.cc` 实现文件并置；统一的
输入、输出、配置和几何类型位于 `src/common/types.h`，日志工具位于
`src/common/logger.h`。保留既有模块层级，例如外部协议适配仍位于
`src/interfaces/`。

`proto/planning/v1/` 定义集成和配置协议，
`config/default_planner_config.textproto` 提供与内部配置类型对应的示例默认值。
生成的 Protobuf C++ 绑定必须输出到构建目录，除非集成需求明确要求，否则不要提交。

核心回归测试位于 `tests/`：`planning_test.cc` 是自包含的可执行测试，
`collision_test_cases.h` 提供其共享碰撞用例。`tools/` 是可选 Qt 6 Widgets
闭环仿真器：`avp_qt_simulator`、其运行时/绘图/场景代码及版本化 JSON 场景均在此处。
启用工具且启用测试时，`avp_simulation_runtime_test` 会注册到 CTest。构建产物只应位于
`build/`，不得提交。

## 规划与安全模型

车辆碰撞统一使用 `common/types.h` 中的 `IsVehicleObstacleCollision`：自车与障碍物均为以
`Pose2d` 为中心的有向矩形，碰撞使用分离轴定理（SAT）判定，边界接触也视为碰撞。
`safety_margin_m` 膨胀自车矩形的半尺寸。新增模块不得退回为圆形或 AABB 近似，也不要在
各模块重复实现碰撞几何。

障碍物预测采用零阶保持：查询时取不晚于查询时刻的最后一帧，早于第一帧时取第一帧。
`local`、`speed` 和顶层最终校验必须使用同一 SAT 矩形碰撞模型。`HybridAStar` 刻意只
使用每个障碍物的第一帧预测；若改变其时域行为，须同步更新 README 和测试。

`local` 使用分层 S-L 格点动态规划。状态保存连续两个横向索引，使扩展至下一层时能将
曲率作为硬约束检查；横向采样范围和分辨率集中在实现中，修改时需补充绕障、锚定自车位姿
和曲率边界测试。`speed` 在时间层上以 `(s, v, a)` 状态和 jerk 候选构建 S-T 动态规划；
速度、加速度、减速度与正负 jerk 都是硬约束，禁止在输出阶段截断来掩盖不可行转移。

`Planner::Plan()` 先适配输入，再由车道拓扑路线、滚动局部参考线和路径—速度交替耦合
生成车道接近轨迹。第一轮按当前时刻规划路径，后续轮将速度剖面回投为路径点到达时刻。
到达入口并停车后，规划器使用 Hybrid A* 生成带方向的 `ParkingManeuver`，按 cusp 分为
单档位段，并在换档前等待停车与驻留时间。局部规划、速度规划或最终校验失败时，返回沿
当前方向匀减速的停车降级轨迹和 `kNoSafeTrajectory`；`stop_collision_free` 诊断只说明
该离散降级轨迹的检查结果，并不保证其一定安全。紧急降级会标记
`jerk_constraint=emergency_exempt`，不受正常舒适性 jerk 约束。

`Planner::Plan(request, PlanningDebugData*)` 会输出本周期可获得的全局路线、裁剪参考线、
路径—速度耦合中间结果、泊车动作和响应，供仿真器及离线诊断使用；不要让核心库依赖 Qt。

## 构建、测试与开发命令

使用标准 CMake 工作流：

```bash
cmake -S . -B build -DAVP_BUILD_TESTS=ON  # 配置核心库和核心测试
cmake --build build --parallel             # 编译；警告视为错误
ctest --test-dir build --output-on-failure # 执行已注册测试
```

可复现的 Debug 构建使用：

```bash
cmake --preset debug
cmake --build --preset debug
ctest --preset debug
```

Qt 闭环仿真器是可选依赖，需要 Qt 6 的 Core 和 Widgets 开发包。使用 `visualize`
预设会在 `build/visualize` 中启用 `AVP_BUILD_VISUALIZERS`；它构建 `avp_qt_simulator`，并在
测试开启时同时注册 `avp_simulation_runtime_test`：

```bash
cmake --preset visualize
cmake --build --preset visualize --target avp_qt_simulator --parallel
ctest --test-dir build/visualize --output-on-failure
./build/visualize/tools/avp_qt_simulator
```

不要让 Qt 成为默认构建或核心测试的依赖。`AVP_ENABLE_CLANG_TIDY` 默认开启；系统找不到
`clang-tidy` 时 CMake 会提示并跳过静态检查。

## Protobuf / protoc 安装

核心库不依赖 Protobuf；仅当需要从 `.proto` 生成 C++ 绑定或接入外部协议时才安装它。在
Debian/Ubuntu 上可运行：

```bash
sudo apt update
sudo apt install protobuf-compiler libprotobuf-dev
protoc --version
```

集成方负责把生成协议及外部数据映射为 `PlanningRequest`、`VehicleConfig` 和
`PlannerConfig`；`default_planner_config.textproto` 当前不会由核心库直接读取。

## 代码风格与命名规范

使用 C++20，遵循仓库的 `.clang-format`（Google 风格、两空格缩进、每行最多 100 列）。
修改 C++ 文件后执行 `clang-format -i <文件>`。CMake 对核心库、测试和工具启用
`-Wall -Wextra -Wpedantic -Werror`；安装并启用的 clang-tidy 会对核心库运行检查。

文件、函数、变量和数据成员使用 `snake_case`（数据成员以 `_` 结尾）；类型使用
`PascalCase`；枚举值和常量使用 `kPascalCase`。工程内包含路径以 `src/` 为根，例如
`#include "speed/speed_planner.h"`。新增公开头文件应与实现放在相同模块目录，并维护头文件
保护或 `#pragma once` 的现有约定。

## 测试指南

每项行为改动都应新增或更新覆盖，尤其是输入校验、规划确定性、路线投影、降级轨迹、
路径—速度耦合、停车换档与边界条件。核心测试使用带显式 `Check` 的小型 `main()`，没有
第三方测试框架；新增核心测试应放在 `tests/planning_test.cc` 或在 `tests/CMakeLists.txt` 中
新增独立 `*_test.cc` 目标并注册 CTest。

涉及安全模型时，至少覆盖有向矩形碰撞、S-L 可行绕障/曲率、S-T 相邻状态满足速度、
加减速度与 jerk 限制，以及顶层最终碰撞校验。修改 Qt 仿真运行时、JSON 场景格式或调试绘图
数据时，应在 `tools/simulation_runtime_test.cc` 补充测试，并以启用 `visualize` 预设的构建
运行该测试。

## 提交与拉取请求指南

请采用简洁的祈使式提交标题，例如 `Add blocked-route fallback test`。每个提交只聚焦一个
变更。拉取请求应说明受影响的规划行为、列出已运行的验证命令并关联相关议题；外部可见
行为变更应附上示例轨迹、诊断输出或场景说明。修改协议字段、默认安全参数、预测语义或
碰撞模型前，必须说明兼容性与验证影响。
