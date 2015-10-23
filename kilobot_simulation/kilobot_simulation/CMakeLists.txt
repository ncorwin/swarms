cmake_minimum_required(VERSION 2.8.3)
project(kilobot_simulation)

find_package(GLUT REQUIRED)
find_package(OpenGL REQUIRED)

add_library(kilobot_simulation_lib robot.cpp)
add_executable(kilobot_simulation main.cpp)
include_directories(${OPENGL_INCLUDE_DIR} ${GLUT_INCLUDE_DIR})
target_link_libraries(kilobot_simulation ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY} ${GLUT_glut_LIBRARY} kilobot_simulation_lib)
