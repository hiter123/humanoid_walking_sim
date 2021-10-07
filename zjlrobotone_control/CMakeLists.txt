cmake_minimum_required(VERSION 3.10)
project(zjlrobotone)

set(CMAKE_CXX_STANDARD 11)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
add_executable(zjlrobotone main.cpp)
target_link_libraries(${PROJECT_NAME}
        ${orocos_kdl_LIBRARIES}
        ${kdl_parser_LIBRARIES}
        Eigen3::Eigen
        )