# 仓库指南

## 项目结构与模块组织

`src/` 按规划阶段划分为 `common`、`interfaces`、`map`、`open_space`、
`local`、`speed` 和 `planning`。每个模块的公开头文件与其 `.cc` 实现文件
放在同一子目录；通用数据类型位于 `src/common/types.h`。保留既有的模块层级，
例如接口适配器仍位于 `src/interfaces/`。

`proto/planning/v1/` 定义集成和配置协议，
`config/default_planner_config.textproto` 提供默认运行参数。将聚焦的
可执行测试放在 `tests/`（`planning_test.cc` 及其共享的
`collision_test_cases.h`）。`tools/` 存放不纳入 CTest 的手动调试工具，当前为
基于 Matplot++ 的 `collision_visualizer`。构建产物应位于 `build/`，不得提交。

## 规划与安全模型

车辆碰撞统一使用 `common/types.h` 中的 `IsVehicleObstacleCollision`：车辆和
障碍物均以 `Pose2d` 为几何中心的有向矩形表示，碰撞通过分离轴定理（SAT）判定。
`safety_margin_m` 膨胀自车矩形的半尺寸；新增规划模块不得退回为圆形或 AABB 近似，
也不要在各模块重复实现碰撞几何。

`local` 使用分层 S-L 格点动态规划。状态保存连续两个横向索引，使曲率可在扩展到
下一层时作为硬约束检查；横向采样范围和分辨率应集中定义，若改动需补充可行绕障与
曲率边界测试。`speed` 使用带前一段速度历史的 S-T 状态；速度、最大加速度和最大
减速度均为硬约束，禁止在输出阶段通过截断加速度来掩盖不可行转移。

障碍物预测采用“取不晚于查询时刻的最后一帧”的零阶保持；查询早于第一帧时使用第一帧。
`local`、`speed` 和顶层最终校验必须使用同一 SAT 矩形碰撞模型。
`HybridAStar` 刻意只使用障碍物第一帧预测；修改其时域行为时应同步更新 README 和测试。

顶层 `Planner::Plan()` 先适配输入，再执行全局路线、路径—速度交替耦合和轨迹合成。
耦合首轮以 `t=0` 规划路径，后续轮次将速度剖面回投为路径点到达时刻。局部规划或最终
校验失败时，返回沿当前航向匀减速的停车降级轨迹和 `kNoSafeTrajectory`；
`stop_collision_free` 诊断仅说明该离散停车轨迹的检查结果，并不保证降级一定安全。

## 构建、测试与开发命令

使用标准 CMake 工作流：

```bash
cmake -S . -B build -DAVP_BUILD_TESTS=ON  # 配置库和测试
cmake --build build --parallel             # 编译；警告视为错误
ctest --test-dir build --output-on-failure # 执行测试套件
```

如需可复现的 Debug 构建，依次运行 `cmake --preset debug`、
`cmake --build --preset debug` 和 `ctest --preset debug`。涉及算法或内存
安全的修改还应运行 `cmake --preset sanitize` 与
`cmake --build --preset sanitize`，然后使用
`ctest --test-dir build/sanitize --output-on-failure`（当前没有 `sanitize` CTest 预设）。

碰撞几何可视化是可选依赖：安装 Matplot++ 和 Gnuplot 后，使用
`cmake --preset visualize`、`cmake --build --preset visualize --target collision_visualizer`
构建，再运行 `./build/visualize/tools/collision_visualizer`。不要让该依赖进入默认构建或测试。

## Protobuf / protoc 安装

当前核心库可在没有 Protobuf 的情况下构建；仅当需要从 `.proto` 生成
C++ 绑定或对接外部协议时才需安装。在 Debian/Ubuntu 上运行：

```bash
sudo apt update
sudo apt install protobuf-compiler libprotobuf-dev
protoc --version
```

这会安装 `protoc` 编译器和 C++ 开发库。其他发行版请安装同名或对应的
`protobuf` / `protobuf-devel` 软件包。生成代码时请将输出放入构建目录，
不要提交生成产物，除非集成需求明确要求。

## 代码风格与命名规范

使用 C++20，并遵循仓库中基于 Google 风格的 `.clang-format`：两空格
缩进、每行最多 100 列。修改 C++ 文件后执行
`clang-format -i <文件>`。安装了 clang-tidy 时会自动运行，且已配置的
告警均视为错误。

文件、函数、变量和数据成员使用 `snake_case`（数据成员以 `_` 结尾）；
类型使用 `PascalCase`；枚举值和常量使用 `kPascalCase`。公开头文件需与
实现文件位于同一目录，例如 `src/speed/speed_planner.h` 与
`src/speed/speed_planner.cc`。工程以内的包含路径以 `src/` 为根，例如
`#include "speed/speed_planner.h"`。

## 测试指南

每项行为改动都应新增或更新测试，尤其是输入校验、规划确定性、降级轨迹
和边界条件。涉及规划安全模型时，至少覆盖矩形车体碰撞、S-L 绕障/曲率可行性及
S-T 相邻速度差满足严格加减速度限制。测试应自包含；当前套件使用带显式检查的
小型 `main()`，没有第三方测试框架。测试文件命名为 `*_test.cc`，并在
`CMakeLists.txt` 中将每个测试可执行文件注册到 CTest。

## 提交与拉取请求指南

请采用简洁的祈使式提交标题，例如 `Add blocked-route fallback test`。每个提交只聚焦一个
变更。拉取请求应说明
受影响的规划行为、列出已运行的验证命令、关联相关议题；若外部可见行为
变更，还应附上示例轨迹或诊断输出。修改协议字段或默认安全参数前，必须
说明兼容性与验证影响。
