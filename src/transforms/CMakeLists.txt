include_directories(.)
add_library(transformation transformation.cc)

# Tests

add_executable(transformation_test transformation_test.cc)
target_link_libraries(transformation_test
                      boost_unit_test_framework
                      transformation)
add_test(TRANSFORMATION_TEST ${CMAKE_CURRENT_BINARY_DIR}/transformation_test)
