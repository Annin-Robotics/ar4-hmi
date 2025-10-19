cmake_minimum_required(VERSION 3.15)
project(robot_kinematics)

set(CMAKE_CXX_STANDARD 14)

find_package(pybind11 REQUIRED)

pybind11_add_module(robot_kinematics bindings.cpp)
