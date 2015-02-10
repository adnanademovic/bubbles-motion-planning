include_directories(.)
add_subdirectory(simple)

add_library(pqp-environment pqp-environment.cc)
target_link_libraries(pqp-environment transformation PQP)

# Tests

add_executable(pqp-environment_test pqp-environment_test.cc)
target_link_libraries(pqp-environment_test pqp-environment boost_unit_test_framework)
add_test(PQP_ENVIRONMENT_TEST ${CMAKE_CURRENT_BINARY_DIR}/pqp-environment_test)
