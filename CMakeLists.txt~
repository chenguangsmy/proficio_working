cmake_minimum_required(VERSION 2.6)
project(Proficio_External)

## proficio_toolbox
find_package(Proficio REQUIRED)
include_directories(${PROFICIO_INCLUDE_DIRS})
link_directories(${PROFICIO_LIBRARY_DIRS})
add_definitions(${PROFICIO_DEFINITIONS} -g -DUSE_LINUX -pthread)

set(GCC_COVERAGE_COMPILE_FLAGS "-frounding-math")
add_definitions(${GCC_COVERAGE_COMPILE_FLAGS})

add_executable(${PROJECT_NAME} proficio_2dBalistic.cpp)
target_link_libraries(${PROJECT_NAME} ${PROFICIO_LIBRARIES} Dragonfly)
set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME "proficio_2dBalistic")

install(TARGETS ${PROJECT_NAME}
        DESTINATION ${CMAKE_CURRENT_SOURCE_DIR}
)

install(CODE "execute_process(COMMAND chown -R ${OWNER}:${OWNER} ${CMAKE_CURRENT_SOURCE_DIR})")
