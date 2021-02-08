cmake_minimum_required(VERSION 3.10)
project(rrt_2d)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(Qt5Widgets REQUIRED)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)


# Set default OpenGL Libraries
set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5 COMPONENTS Widgets OpenGL REQUIRED)
find_package(box2d REQUIRED) 
ADD_SUBDIRECTORY (googletest)
enable_testing()

include_directories(
    ${gtest_SOURCE_DIR}/include
    ${gtest_SOURCE_DIR}
    ${BOX2D_INCLUDE_DIRECTORIES}
    include/
)



set(SOURCES
        ui/glwidget.cpp
        ui/helper.cpp
        src/main.cpp
        ui/mainwindow.cpp
        ui/mainwindow.ui
        ui/widget.cpp
        src/Utils.cpp 
        src/GridMap.cpp
)

add_executable(rrt_2d ${SOURCES})
target_link_libraries(rrt_2d  Qt5::Widgets  Qt5::OpenGL ${OPENGL_LIBRARIES} Qt5::Core box2d)

################################
# Unit Tests
################################
# Add test cpp file
add_executable(runUnitTests Test/UtilsTest.cpp src/Utils.cpp Test/GridMapTest.cpp src/GridMap.cpp Test/OctreeTest.cpp)
target_link_libraries(runUnitTests gtest gtest_main box2d)
add_test( runUnitTests runUnitTests )

# Test the Octree
add_executable(runOctreeTests Test/OctreeTest.cpp)
target_link_libraries(runOctreeTests gtest gtest_main)
add_test(runOctreeTests runOctreeTests)
