cmake_minimum_required(VERSION 3.7)
project(FinalProject)

set(CMAKE_CXX_STANDARD 11)

find_package(MPI REQUIRED)
find_package(OpenGL REQUIRED)
find_package(freeglut REQUIRED)
find_package(GLUT REQUIRED)

set(SOURCE_FILES main.cpp GLAD/src/glad.c ECE_UAV.cpp)
set(HEADER_FILES shader.h BMP.h ECE_UAV.h)

set(dir ${CMAKE_CURRENT_SOURCE_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${dir}/build)

include_directories(include GL)
include_directories(SYSTEM ${MPI_INCLUDE_PATH})

#Include library files
include_directories(GLAD/include)
include_directories(glm/include)
include_directories(${OPENGL_INCLUDE_DIRS} ${GLUT_INCLUDE_DIRS})

file(COPY resources DESTINATION ${dir}/build)
file(COPY shaders DESTINATION ${dir}/build)


#Add deps
add_subdirectory(dependencies)

add_executable(FinalProject ${SOURCE_FILES} ${HEADER_FILES})
target_link_libraries(FinalProject glfw ${MPI_C_LIBRARIES} ${OPENGL_LIBRARIES} ${GLUT_LIBRARY})





