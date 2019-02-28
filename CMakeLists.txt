cmake_minimum_required(VERSION 3.10)
project(thesis)

set(CMAKE_CXX_STANDARD 14)

#set(CMAKE_C_FLAGS "-O2 -g -ggdb -Wall -fstack-protector-all")
SET(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-pthread")

include(FindProtobuf)
find_package(Protobuf REQUIRED)
include_directories(${Protobuf_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS
        proto/vehicle_state.proto
        proto/environment_conf.proto
        proto/prediction_obstacles.proto
        proto/speed_profile_conf.proto
        proto/trajectory.proto
        proto/path_planner_debug.proto
        proto/path_planner_conf.proto
        proto/planning_conf.proto
        proto/planning_debug.proto
        proto/speed_profile_debug.proto)

add_library(planning_proto ${PROTO_HDRS} ${PROTO_SRCS})

SET(OpenCV_DIR /usr/local/Cellar/opencv@3/3.4.5/share/OpenCV)
find_package(OpenCV)

include_directories(
        ${OpenCV_INCLUDE_DIRS}
)

add_library(planner
            src/common/reference_route.cpp
            src/common/reference_route.h
            src/common/static_obstacles.cpp
            src/common/static_obstacles.h
            src/common/reference_path.cpp
            src/common/reference_path.h
            src/frame/frame.h
            src/frame/structured_frame.cpp
            src/frame/structured_frame.h
            src/frame/unstructured_frame.cpp
            src/frame/unstructured_frame.h
            src/frame/frenet_frame.cpp
            src/frame/frenet_frame.h
            src/path_planner/image_proc.cpp
            src/path_planner/image_proc.h
            src/path_planner/searching_frame.cpp
            src/path_planner/searching_frame.h
            src/path_planner/probablistic_map.cpp
            src/path_planner/probablistic_map.h
            src/path_planner/heuristic_rrt.cpp
            src/path_planner/heuristic_rrt.h
            src/path_planner/node.h
            src/path_planner/gnat.cpp
            src/path_planner/gnat.h
            src/path_planner/path_planner_record.cpp
            src/planner.cpp
            src/planner.h
            src/utils/file_config.h
            src/utils/string.h
            src/utils/spline.cpp
            src/utils/spline.h
            src/utils/timer.h
            src/speed_planner/dynamic_obstacles.cpp
            src/speed_planner/dynamic_obstacles.h
            src/speed_planner/rrt.cpp
            src/speed_planner/rrt.h
            src/speed_planner/node.h
            src/speed_planner/speed_profile_record.cpp
)

target_link_libraries(planner
        ${OpenCV_LIBRARIES}
        ${PROTOBUF_LIBRARY}
        planning_proto)


add_executable(thesis src/main.cpp)
target_link_libraries(thesis
        ${OpenCV_LIBRARIES}
        planner
        planning_proto
        ${PROTOBUF_LIBRARY}
        )
