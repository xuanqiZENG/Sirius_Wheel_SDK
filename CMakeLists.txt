cmake_minimum_required(VERSION 3.5)
project(USB2CAN_Linux)

set(CMAKE_CXX_STANDARD 14)

set(INC_DIR /usr/local/include)
set(LINK_DIR /usr/local/lib)

set(CMAKE_CXX_STANDARD 14)
include_directories("/usr/local/include/eigen3")   
include_directories("./lcm-types/cpp") # lcm includes
include_directories(${INC_DIR})
include_directories("./include/")
file(GLOB_RECURSE sources
"./src/*.cpp")

link_directories(${LINK_DIR})
add_executable(USB2CAN_Linux main.cpp ${sources} )
target_link_libraries(USB2CAN_Linux libusb-1.0.so pthread lcm)
