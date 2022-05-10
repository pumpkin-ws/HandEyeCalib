find_package(OpenCV 4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/bin/)

add_executable(cv_yaml cv_yaml.cpp)
target_link_libraries(cv_yaml ${OpenCV_LIBS})

add_executable(rot_vec rot_vec.cpp)
target_link_libraries(rot_vec ${OpenCV_LIBS})

add_executable(eigen_test eigen_test.cpp)
target_link_libraries(eigen_test ${OpenCV_LIBS})

add_executable(convert_pose convert_pose.cpp)
target_link_libraries(convert_pose ${OpenCV_LIBS})