include_directories(.)

add_library(pqp-environment pqp-environment.cc)
target_link_libraries(pqp-environment transformation PQP boost_serialization)
add_library(pqp-environment-feedback pqp-environment-feedback.cc)
target_link_libraries(pqp-environment-feedback pqp-environment bubble)

# Tests

add_executable(pqp-environment_test pqp-environment_test.cc)
target_link_libraries(pqp-environment_test pqp-environment boost_unit_test_framework)
add_test(PQP_ENVIRONMENT_TEST ${CMAKE_CURRENT_BINARY_DIR}/pqp-environment_test)

add_executable(pqp-environment-feedback_test pqp-environment-feedback_test.cc)
target_link_libraries(pqp-environment-feedback_test pqp-environment-feedback boost_unit_test_framework)
add_test(PQP_ENVIRONMENT_FEEDBACK_TEST ${CMAKE_CURRENT_BINARY_DIR}/pqp-environment-feedback_test)
