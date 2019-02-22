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
        proto/planning_debug.proto
        proto/path_planner_conf.proto)

add_library(planning_proto ${PROTO_HDRS} ${PROTO_SRCS})

find_package(OpenCV)

include_directories(
        ${OpenCV_INCLUDE_DIRS}
)

add_library(path_planner
            src/common/reference_route.cpp
            src/common/reference_route.h
            src/common/obstacles.cpp
            src/common/obstacles.h
            src/common/spline.h
            src/common/spline.cpp
            src/frame/frame.h
            src/frame/structured_frame.cpp
            src/frame/structured_frame.h
            src/frame/unstructured_frame.cpp
            src/frame/unstructured_frame.h
            src/frame/frenet_frame.cpp
            src/frame/frenet_frame.h
            src/path_planner/path_planner.cpp
            src/path_planner/path_planner.h
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
            src/path_planner/gnat.h)

target_link_libraries(path_planner
        ${OpenCV_LIBRARIES}
        ${PROTOBUF_LIBRARY}
        planning_proto)


add_executable(thesis src/main.cpp)
target_link_libraries(thesis
        ${OpenCV_LIBRARIES}
        path_planner
        planning_proto
        ${PROTOBUF_LIBRARY}
        )
