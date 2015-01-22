include_directories(.)
add_subdirectory(environment)
add_subdirectory(generators)
add_subdirectory(transforms)

add_library(bubble bubble.cc)
target_link_libraries(bubble tree-node)
add_library(bubble-rrt bubble-rrt.cc)
target_link_libraries(bubble-rrt bubble-tree)
add_library(bubble-tree bubble-tree.cc)
target_link_libraries(bubble-tree bubble flann_cpp)
add_library(tree-node tree-node.cc)

add_executable(example-main example-main.cc)
target_link_libraries(example-main
                      bubble-rrt
                      bubble-source
                      bubble-tree
                      obstacle-rectangle
                      obstacle-sphere
                      abb-irb120-manipulator
                      simple-generator)

# Tests

add_executable(bubble_test bubble_test.cc)
target_link_libraries(bubble_test bubble boost_unit_test_framework)
add_test(BUBBLE_TEST ${CMAKE_CURRENT_BINARY_DIR}/bubble_test)

