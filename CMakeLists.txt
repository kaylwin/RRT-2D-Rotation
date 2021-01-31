cmake_minimum_required(VERSION 3.10)
project(Test)

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


#####################################################
#           Set up Box2d
#####################################################
set(box2d_libs
        libbox2d.a
        libglad.a
        libglfw.a
        libimgui.a
        libsajson.a
)
include_directories(box_2d/include)
link_directories(box_2d/libs)

set(SOURCES
        ui/glwidget.cpp
        ui/helper.cpp
        main.cpp
        ui/mainwindow.cpp
        ui/mainwindow.ui
        ui/widget.cpp
)

add_executable(helloworld ${SOURCES})
target_link_libraries(helloworld Qt5::Widgets  Qt5::OpenGL ${OPENGL_LIBRARIES} Qt5::Core ${box2d_libs})