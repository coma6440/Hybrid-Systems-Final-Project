cmake_minimum_required(VERSION 3.0.0)
project(HybridSystemsProject)

find_package(ompl REQUIRED)

include_directories(
	${OMPL_INCLUDE_DIRS}
	${PROJECT_SOURCE_DIR}/include
)

add_executable(main src/main.cpp include/json.hpp)

target_link_libraries(main ${OMPL_LIBRARIES})