cmake_minimum_required(VERSION 3.0.0)
project(aruco VERSION 0.1.0)

find_package(OpenCV REQUIRED)
include_directories( "/home/joker/eigen" )

#从main.cpp生成可执行文件ArUco_Test
add_executable(aruco0 aruco.cpp)
add_executable(hh hh.cpp)



#生成文件链接至OpenCV库，注意OpenCV_LIBS变量需在OpenCVConfig.cmake文件中定义，下同
target_link_libraries(aruco0 ${OpenCV_LIBS} )
target_link_libraries(hh ${OpenCV_LIBS} )



#采用c++11标准编译，没有此句会报错
set(CMAKE_CXX_FLAGS "-std=c++11")
