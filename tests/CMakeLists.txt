add_executable(avp_planning_test planning_test.cc)
target_include_directories(avp_planning_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(avp_planning_test PRIVATE avp::planning spdlog::spdlog)
target_compile_options(avp_planning_test PRIVATE -Wall -Wextra -Wpedantic -Werror)

add_test(NAME avp_planning_test COMMAND avp_planning_test)
