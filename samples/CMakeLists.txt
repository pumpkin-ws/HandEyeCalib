cmake_minimum_required(VERSION 3.5)
project(sparvis-test)

find_package(OpenCV 4 REQUIRED)

add_executable(cam_calib cam_calib.cpp)
target_link_libraries(cam_calib sparkvis ${OpenCV_LIBS})

add_executable(eih_calib eih_calib.cpp)
target_link_libraries(eih_calib sparkvis ${OpenCV_LIBS})

add_executable(eth_calib eih_calib.cpp)
target_link_libraries(eth_calib sparkvis ${OpenCV_LIBS})

