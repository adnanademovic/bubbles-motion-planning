include_directories(.)
add_subdirectory(environment)
add_subdirectory(transforms)

add_library(bubble bubble.cc)
add_library(bubble-tree bubble-tree.cc)
target_link_libraries(bubble-tree bubble)
add_library(bubble-rrt bubble-rrt.cc)
target_link_libraries(bubble-rrt bubble-tree)

# Tests

add_executable(bubble_test bubble_test.cc)
target_link_libraries(bubble_test bubble boost_unit_test_framework)
add_test(BUBBLE_TEST ${CMAKE_CURRENT_BINARY_DIR}/bubble_test)

