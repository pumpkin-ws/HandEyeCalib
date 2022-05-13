cmake_minimum_required(VERSION 3.0.0)
project(SparkHandEye VERSION 0.1.0)

include(CTest)
enable_testing()
add_compile_options(-std=c++14)

add_compile_options(-std=c++14)

find_package(OpenCV 4 REQUIRED)

link_directories(${PROJECT_SOURCE_DIR}/lib/)
include_directories(${PROJECT_SOURCE_DIR}/source/)

add_subdirectory(${PROJECT_SOURCE_DIR}/source/)
add_subdirectory(${PROJECT_SOURCE_DIR}/examples/)
add_subdirectory(${PROJECT_SOURCE_DIR}/validations/)

add_executable(SparkHandEye main.cpp)
target_link_libraries(SparkHandEye spk ${OpenCV_LIBS})

install(TARGETS sparkvis
LIBRARY DESTINATION /usr/lib/
)

install(
    DIRECTORY source/
    DESTINATION /usr/include/sparkvis/
    FILES_MATCHING PATTERN "*.hpp"
)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
