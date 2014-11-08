include_directories(.)
add_library(obstacle-sphere obstacle-sphere.cc)
add_library(obstacle-rectangle obstacle-rectangle.cc)

# Tests

add_executable(obstacle-sphere_test obstacle-sphere_test.cc)
target_link_libraries(obstacle-sphere_test obstacle-sphere boost_unit_test_framework)
add_test(OBSTACLE_SPHERE_TEST ${CMAKE_CURRENT_BINARY_DIR}/obstacle-sphere_test)

add_executable(obstacle-rectangle_test obstacle-rectangle_test.cc)
target_link_libraries(obstacle-rectangle_test obstacle-rectangle boost_unit_test_framework)
add_test(OBSTACLE_RECTANGLE_TEST ${CMAKE_CURRENT_BINARY_DIR}/obstacle-rectangle_test)
