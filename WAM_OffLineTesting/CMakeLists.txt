cmake_minimum_required(VERSION 2.6)
project(Proficio_External)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wall -Wextra")

## proficio_toolbox
find_package(Barrett REQUIRED)
include_directories(${BARRETT_INCLUDE_DIRS} ${RTMA_INCLUDES_DIR})
link_directories(${BARRETT_LIBRARY_DIRS} /usr/bin/)
add_definitions(${BARRETT_DEFINITIONS} -g -DUSE_LINUX -pthread)

## curses
find_package(Curses REQUIRED)
include_directories(${CURSES_INCLUDE_DIR})


set(GCC_COVERAGE_COMPILE_FLAGS "-frounding-math")
add_definitions(${GCC_COVERAGE_COMPILE_FLAGS})

add_executable(${PROJECT_NAME} CustomController.cpp)
target_link_libraries(${PROJECT_NAME} ${BARRETT_LIBRARIES} RTMA )
set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME "CustomController")

install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}
)

install(CODE "execute_process(COMMAND chown -R ${OWNER}:${OWNER} ${CMAKE_CURRENT_SOURCE_DIR})")
