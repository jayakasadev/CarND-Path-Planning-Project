project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources src/main.cpp src/utilities/utilities.h src/utilities/constants.h src/trajectory_generation/trajectory.cpp src/trajectory_generation/trajectory.h src/sensor_fusion/sensor_fusion.cpp src/sensor_fusion/sensor_fusion.h src/utilities/map.h src/utilities/vehicle.h src/utilities/car_state.h src/trajectory_generation/jerk_minimizer.cpp src/trajectory_generation/jerk_minimizer.h)


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
