find_package(Qt5 REQUIRED COMPONENTS Gui Test)

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
