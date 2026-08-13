find_package(Qt5 5.15 REQUIRED COMPONENTS Core Widgets)

add_executable(avp_qt_simulator
  plot_data.cc
  plot_data.h
  qt_simulator_main.cc
  scenario.cc
  scenario.h
  simulation_runtime.cc
  simulation_runtime.h
)
set_target_properties(avp_qt_simulator PROPERTIES AUTOMOC ON)
target_include_directories(avp_qt_simulator PRIVATE ${PROJECT_SOURCE_DIR})
target_link_libraries(avp_qt_simulator PRIVATE avp::planning Qt5::Core Qt5::Widgets)
target_compile_options(avp_qt_simulator PRIVATE -Wall -Wextra -Wpedantic -Werror)

if(AVP_BUILD_TESTS)
  add_executable(avp_simulation_runtime_test
    plot_data.cc
    plot_data.h
    simulation_runtime_test.cc
    scenario.cc
    scenario.h
    simulation_runtime.cc
    simulation_runtime.h
  )
  target_include_directories(avp_simulation_runtime_test PRIVATE ${PROJECT_SOURCE_DIR})
  target_link_libraries(avp_simulation_runtime_test PRIVATE avp::planning Qt5::Core)
  target_compile_definitions(avp_simulation_runtime_test
    PRIVATE AVP_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
  target_compile_options(avp_simulation_runtime_test PRIVATE -Wall -Wextra -Wpedantic -Werror)
  add_test(NAME avp_simulation_runtime_test COMMAND avp_simulation_runtime_test)
endif()
