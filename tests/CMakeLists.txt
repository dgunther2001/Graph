project(graph)

set(EXECUTABLE_OUTPUT_PATH ../)

add_executable(test main.cpp)

target_link_libraries(test graph)