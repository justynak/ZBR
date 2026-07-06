find_package(Qt5 REQUIRED COMPONENTS Gui Widgets Test)

add_executable(tst_kinematicpoints
    tst_kinematicpoints.cpp
    ${CMAKE_SOURCE_DIR}/kinematicpoints.cpp
    ${CMAKE_SOURCE_DIR}/kinematicpoints.h
)
target_include_directories(tst_kinematicpoints PRIVATE ${CMAKE_SOURCE_DIR})
target_link_libraries(tst_kinematicpoints PRIVATE Qt5::Gui Qt5::Test)
add_test(NAME kinematicpoints COMMAND tst_kinematicpoints)

add_executable(tst_trajectorypoints
    tst_trajectorypoints.cpp
    ${CMAKE_SOURCE_DIR}/trajectorypoints.cpp
    ${CMAKE_SOURCE_DIR}/trajectorypoints.h
)
target_include_directories(tst_trajectorypoints PRIVATE ${CMAKE_SOURCE_DIR})
target_link_libraries(tst_trajectorypoints PRIVATE Qt5::Gui Qt5::Test)
add_test(NAME trajectorypoints COMMAND tst_trajectorypoints)

add_executable(tst_widgetanimation
    tst_widgetanimation.cpp
    ${CMAKE_SOURCE_DIR}/widgetanimation.cpp
    ${CMAKE_SOURCE_DIR}/widgetanimation.h
    ${CMAKE_SOURCE_DIR}/widgetanimation.ui
    ${CMAKE_SOURCE_DIR}/kinematicpoints.cpp
    ${CMAKE_SOURCE_DIR}/kinematicpoints.h
    ${CMAKE_SOURCE_DIR}/trajectorypoints.cpp
    ${CMAKE_SOURCE_DIR}/trajectorypoints.h
)
target_include_directories(tst_widgetanimation PRIVATE ${CMAKE_SOURCE_DIR})
target_link_libraries(tst_widgetanimation PRIVATE Qt5::Widgets Qt5::Test)
add_test(NAME widgetanimation COMMAND tst_widgetanimation)
# no display on CI runners
set_tests_properties(widgetanimation PROPERTIES ENVIRONMENT "QT_QPA_PLATFORM=offscreen")
