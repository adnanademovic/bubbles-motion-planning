include_directories(.)
add_subdirectory(environment)
add_subdirectory(transforms)

add_library(bubble bubble.cc)
