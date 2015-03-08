include_directories(.)

add_library(pqp-environment pqp-environment.cc)
target_link_libraries(pqp-environment
                      boost_filesystem
                      boost_serialization
                      boost_system
                      boost_thread
                      PQP
                      transformation)

add_library(pqp-environment-feedback pqp-environment-feedback.cc)
target_link_libraries(pqp-environment-feedback
                      bubble
                      pqp-environment)

# Tests

add_executable(pqp-environment_test pqp-environment_test.cc)
target_link_libraries(pqp-environment_test
                      boost_unit_test_framework
                      pqp-environment)
add_test(PQP_ENVIRONMENT_TEST ${CMAKE_CURRENT_BINARY_DIR}/pqp-environment_test)

add_executable(pqp-environment-feedback_test pqp-environment-feedback_test.cc)
target_link_libraries(pqp-environment-feedback_test
                      boost_unit_test_framework
                      pqp-environment-feedback)
add_test(PQP_ENVIRONMENT_FEEDBACK_TEST
         ${CMAKE_CURRENT_BINARY_DIR}/pqp-environment-feedback_test)
