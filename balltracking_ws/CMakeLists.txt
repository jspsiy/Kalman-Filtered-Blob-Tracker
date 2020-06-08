# stop if cmake version is below 3.0
cmake_minimum_required(VERSION 3.0 )

# project name and enable C++ support
project(ball_tacker)

find_package( OpenCV REQUIRED COMPONENTS )

include_directories(
include
)
add_compile_options(-std=c++17)
set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-fmax-errors=5 -std=c++17 -pthread -Wfatal-errors ")
# require C++11


# we define the executable

add_executable(ball_tracker src/balltrack.cpp src/KalmanObject.cpp)
target_link_libraries(ball_tracker ${OpenCV_LIBS})

