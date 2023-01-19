cmake_minimum_required(VERSION 3.24)
project(SpringKit)

set(CMAKE_CXX_STANDARD 17)

add_executable(SpringKit main.cpp SpringForce.cpp SpringForce.h SpringAnimation.cpp SpringAnimation.h)
