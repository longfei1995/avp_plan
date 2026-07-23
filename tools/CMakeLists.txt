find_package(Matplot++ CONFIG REQUIRED)

find_program(AVP_GNUPLOT_EXECUTABLE NAMES gnuplot)
if(NOT AVP_GNUPLOT_EXECUTABLE)
  message(FATAL_ERROR
    "AVP_BUILD_VISUALIZERS requires Gnuplot for the Matplot++ backend. "
    "Install gnuplot and ensure it is available on PATH.")
endif()

add_executable(collision_visualizer collision_visualizer.cc)
target_include_directories(collision_visualizer PRIVATE ${PROJECT_SOURCE_DIR}/tests)
target_link_libraries(collision_visualizer PRIVATE avp::planning Matplot++::matplot)
target_compile_options(collision_visualizer PRIVATE -Wall -Wextra -Wpedantic -Werror)
