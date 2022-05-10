set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/bin/)

add_executable(cam_calib cam_calib.cpp)
target_link_libraries(cam_calib sparkvis)

add_executable(eih_calib eih_calib.cpp)
target_link_libraries(eih_calib sparkvis)

add_executable(eth_calib eth_calib.cpp)
target_link_libraries(eth_calib sparkvis)