cmake_minimum_required(VERSION 3.22)
project(KalmanFilter)

set(CMAKE_CXX_STANDARD 20)

add_executable(KalmanFilter main.cpp kalmanFilter/kalmanFilter.cpp kalmanFilter/kalmanFilter.h kalmanFilter/Matrix.h kalmanFilter/DMatrix.h kalmanFilter/invert.h)
