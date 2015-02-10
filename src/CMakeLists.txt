include_directories(.)
add_subdirectory(environment)
add_subdirectory(generators)
add_subdirectory(transforms)

add_library(bubble bubble.cc)
target_link_libraries(bubble tree-node)
add_library(rrt rrt.cc)
target_link_libraries(rrt rrt-tree)
add_library(bubble-tree bubble-tree.cc)
target_link_libraries(bubble-tree bubble rrt-tree)
add_library(classic-tree classic-tree.cc)
target_link_libraries(classic-tree rrt-tree)
add_library(point-index point-index.cc)
target_link_libraries(point-index tree-node flann_cpp)
add_library(rrt-tree rrt-tree.cc)
target_link_libraries(rrt-tree point-index)
add_library(tree-node tree-node.cc)

add_executable(twoseg-bubble-main twoseg-bubble-main.cc)
target_link_libraries(twoseg-bubble-main
                      bubble-tree
                      pqp-environment
                      pqp-environment-feedback
                      rrt
                      simple-generator)

add_executable(bubble-main bubble-main.cc)
target_link_libraries(bubble-main
                      abb-irb120-manipulator
                      bubble-source
                      bubble-tree
                      obstacle-rectangle
                      obstacle-sphere
                      rrt
                      simple-generator)

add_executable(classic-main classic-main.cc)
target_link_libraries(classic-main
                      abb-irb120-manipulator
                      bubble-source
                      classic-tree
                      obstacle-rectangle
                      obstacle-sphere
                      rrt
                      simple-generator)

add_executable(compare-main compare-main.cc)
target_link_libraries(compare-main
                      abb-irb120-manipulator
                      bubble-source
                      bubble-tree
                      classic-tree
                      obstacle-rectangle
                      obstacle-sphere
                      rrt
                      simple-generator)

# Tests

add_executable(bubble_test bubble_test.cc)
target_link_libraries(bubble_test bubble boost_unit_test_framework)
add_test(BUBBLE_TEST ${CMAKE_CURRENT_BINARY_DIR}/bubble_test)

