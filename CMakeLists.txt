cmake_minimum_required(VERSION 3.16)
project(ZBRy LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(Qt5 REQUIRED COMPONENTS Widgets)

add_executable(ZBRy
    main.cpp
    mainwindow.cpp
    mainwindow.h
    mainwindow.ui
    widgetanimation.cpp
    widgetanimation.h
    widgetanimation.ui
    widgetsettings.cpp
    widgetsettings.h
    widgetsettings.ui
    kinematicpoints.cpp
    kinematicpoints.h
    trajectorypoints.cpp
    trajectorypoints.h
)

target_link_libraries(ZBRy PRIVATE Qt5::Widgets)

enable_testing()
add_subdirectory(tests)
