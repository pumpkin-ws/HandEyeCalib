find_package(OpenCV 4 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${SparkHandEye_SOURCE_DIR}/lib/)

find_package(Boost REQUIRED filesystem)

add_library(
    sparkvis SHARED
    camera_intrinsics.hpp
    camera_intrinsics.cpp
    eye_in_hand.hpp
    eye_in_hand.cpp
    eye_to_hand.hpp
    eye_to_hand.cpp
    file_manip.hpp
    file_manip.cpp
)

target_link_libraries(sparkvis ${OpenCV_LIBS} ${Boost_LIBRARIES})

