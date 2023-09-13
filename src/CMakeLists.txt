find_library(PLANNER_LIB libgsmpl.a PATHS ~/ws_moveit2/install/gsmpl/lib)
message(${PLANNER_LIB})
file(GLOB_RECURSE SOURCE_FILES "*.cpp" "collision_checker/*.cpp" 
    "local_planner/*.cpp" "robot_algo/*.cpp" "visualizer/*.cpp")
aux_source_directory(. DIR_LIB_SRCS)
message("DIR_LIB_SRCS" ${DIR_LIB_SRCS})
add_library(rrt_plugin ${SOURCE_FILES})
# add Planner lib
target_link_libraries(rrt_plugin ${PLANNER_LIB})
include_directories(~/ws_moveit2/install/gsmpl/include)
set(INCLUDE_DIRS
    collision_checker
    local_planner
    robot_algo
    visualizer
)
include_directories(${INCLUDE_DIRS})

ament_target_dependencies(rrt_plugin ${PACKAGE_INCLUDE_DEPENDENCIES})

set(INSTALL_DIR ~/ws_moveit2/install/rrt_plugin)

install(TARGETS rrt_plugin EXPORT export_${PROJECT_NAME} LIBRARY DESTINATION ${INSTALL_DIR}/lib)

install(TARGETS rrt_plugin 
  EXPORT moveit_planners_rrt_pluginTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
ament_export_targets(moveit_planners_rrt_pluginTargets HAS_LIBRARY_TARGET)

foreach(dir ${INCLUDE_DIRS})
    file(GLOB_RECURSE headers "${dir}/*.h")
    install(FILES ${headers} DESTINATION ${INSTALL_DIR}/incldue/${dir})
endforeach()

install(FILES rrt_planning_context.h DESTINATION ${INSTALL_DIR}/incldue/${dir})
install(FILES visibility_control.h DESTINATION ${INSTALL_DIR}/incldue/${dir})

ament_export_libraries(rrt_plugin)
ament_export_targets(export_${PROJECT_NAME})

# demo
add_executable(rrt_demo rrt_demo.cpp)

# find_library(PLANNER_LIB libPlanner.so PATHS lib)
target_link_libraries(rrt_demo ${PLANNER_LIB})
ament_target_dependencies(rrt_demo ${PACKAGE_INCLUDE_DEPENDENCIES})

install(TARGETS rrt_demo DESTINATION ${INSTALL_DIR}/lib/${PROJECT_NAME})
install(TARGETS rrt_demo
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)