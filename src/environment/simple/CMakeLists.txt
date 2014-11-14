include_directories(.)
add_library(obstacle-sphere obstacle-sphere.cc)
add_library(obstacle-rectangle obstacle-rectangle.cc)
add_library(planar-2seg-manipulator planar-2seg-manipulator.cc)
add_library(bubble-source bubble-source.cc)

# Tests

add_executable(obstacle-sphere_test obstacle-sphere_test.cc)
target_link_libraries(obstacle-sphere_test obstacle-sphere boost_unit_test_framework)
add_test(OBSTACLE_SPHERE_TEST ${CMAKE_CURRENT_BINARY_DIR}/obstacle-sphere_test)

add_executable(obstacle-rectangle_test obstacle-rectangle_test.cc)
target_link_libraries(obstacle-rectangle_test obstacle-rectangle boost_unit_test_framework)
add_test(OBSTACLE_RECTANGLE_TEST ${CMAKE_CURRENT_BINARY_DIR}/obstacle-rectangle_test)

add_executable(planar-2seg-manipulator_test planar-2seg-manipulator_test.cc)
target_link_libraries(planar-2seg-manipulator_test planar-2seg-manipulator boost_unit_test_framework)
add_test(PLANAR_2SEG_MANIPULATOR_TEST ${CMAKE_CURRENT_BINARY_DIR}/planar-2seg-manipulator_test)

add_executable(bubble-source_test bubble-source_test.cc)
target_link_libraries(bubble-source_test bubble-source boost_unit_test_framework)
add_test(BUBBLE_SOURCE_TEST ${CMAKE_CURRENT_BINARY_DIR}/bubble-source_test)
