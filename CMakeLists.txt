cmake_minimum_required(VERSION 3.28)
project(avp_planning VERSION 0.1.0 LANGUAGES CXX)


# ========== 构建选项（可通过命令行 -D 或 CMakePresets 覆盖） ==========
# option 定义布尔型缓存开关，格式：option(变量名 说明文字 默认值)
option(AVP_BUILD_TESTS "构建规划器测试" ON)
option(AVP_BUILD_VISUALIZERS "构建规划器可视化调试工具" OFF)
option(AVP_ENABLE_SANITIZERS "启用地址与未定义行为消毒器" OFF)
option(AVP_ENABLE_CLANG_TIDY "安装 clang-tidy 时运行静态检查" ON)

add_library(avp_planning
  src/common/types.h
  src/interfaces/adapters.cc
  src/interfaces/adapters.h
  src/map/global_planner.cc
  src/map/global_planner.h
  src/open_space/hybrid_a_star.cc
  src/open_space/hybrid_a_star.h
  src/local/local_planner.cc
  src/local/local_planner.h
  src/speed/speed_planner.cc
  src/speed/speed_planner.h
  src/planning/planner.cc
  src/planning/planner.h
)

# 给目标创建别名，避免命名冲突
# add_library(<alias> ALIAS <target>)
add_library(avp::planning ALIAS avp_planning)

target_include_directories(avp_planning PUBLIC src)
target_compile_features(avp_planning PUBLIC cxx_std_20)
target_compile_options(avp_planning PRIVATE -Wall -Wextra -Wpedantic -Werror)


# ========== clang-tidy 静态代码检查 ==========
if(AVP_ENABLE_CLANG_TIDY)
  find_program(AVP_CLANG_TIDY_EXECUTABLE clang-tidy)
  if(AVP_CLANG_TIDY_EXECUTABLE)
    set_property(TARGET avp_planning PROPERTY CXX_CLANG_TIDY "${AVP_CLANG_TIDY_EXECUTABLE};--warnings-as-errors=*")
  else()
    message(STATUS "clang-tidy not found; static analysis is disabled for this build")
  endif()
endif()


# ========== Sanitizer 动态运行时检查 ==========
# 仅在 GCC / Clang 编译器下生效（MSVC 语法不同）
if(AVP_ENABLE_SANITIZERS AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(avp_planning PUBLIC -fsanitize=address,undefined -fno-omit-frame-pointer)
  target_link_options(avp_planning PUBLIC -fsanitize=address,undefined)
endif()

# ========== 单元测试 ==========
if(AVP_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()

# ========== 可视化调试工具 ==========
if(AVP_BUILD_VISUALIZERS)
  add_subdirectory(tools)
endif()
