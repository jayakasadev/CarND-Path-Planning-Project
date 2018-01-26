project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources src/main.cpp src/trajectory_generation/map/map.h src/utilities/utilities.h src/behavior_planner/behavior_planner.cpp src/behavior_planner/behavior_planner.h src/sensor_fusion/sensor_fusion.cpp src/sensor_fusion/sensor_fusion.h src/trajectory_generation/trajectory_generator.cpp src/trajectory_generation/trajectory_generator.h src/vehicle/vehicle.h src/utilities/read_write_lock.h src/vehicle/driver.cpp src/vehicle/driver.h src/vehicle/traffic.cpp src/vehicle/traffic.h src/behavior_planner/planning/highway_planner.cpp src/behavior_planner/planning/highway_planner.h src/behavior_planner/planning/city_planner.cpp src/behavior_planner/planning/city_planner.h src/behavior_planner/planning/planner.h src/behavior_planner/trajectory_calculator/trajectory_calculator.h src/tunable_params/sensor_fusion_tunable.h src/tunable_params/behavior_planning_tunable.h src/constants/simulator_constants.h src/behavior_planner/trajectory_calculator/trajectory_calculator.cpp src/trajectory/trajectory.h src/enums/trajectory_type.h src/trajectory/trajectory.cpp src/cost_functions/cost_function.h src/cost_functions/cost_function.cpp src/cost_functions/drive_mode/drive_modes.h src/behavior_planner/behavior_planner_factory.cpp src/behavior_planner/behavior_planner_factory.h src/utilities/unique_ptr_helper.h src/tunable_params/cost_function_tunable.h src/utilities/pools/object_pool.h src/utilities/pools/pointer_pool.h src/utilities/pools/synch_pool.h src/behavior_planner/points/points.cpp src/behavior_planner/points/points.h)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 


add_executable(path_planning ${sources})

# added -lpthread to the end to allow program to be multithreaded
target_link_libraries(path_planning z ssl uv uWS -lpthread)
