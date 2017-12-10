project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources src/main.cpp src/map/map.h src/constants/constants.h src/utilities/utilities.h src/behavior_planner/behavior_planner.cpp src/behavior_planner/behavior_planner.h src/sensor_fusion/sensor_fusion.cpp src/sensor_fusion/sensor_fusion.h src/trajectory_generation/trajectory_generator.cpp src/trajectory_generation/trajectory_generator.h src/vehicle/vehicle.h src/enums/vehicle_behavior.h src/enums/drive_mode.h src/scores/scores.cpp src/scores/scores.h src/utilities/read_write_lock.h src/vehicle/driver.cpp src/vehicle/driver.h src/vehicle/traffic.cpp src/vehicle/traffic.h)


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
