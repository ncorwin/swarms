cmake_minimum_required(VERSION 2.8.3)
project(KilobotSimulator)

find_package(GLUT REQUIRED)
find_package(OpenGL REQUIRED)

add_library(KilobotSimulator robot.cpp)
add_executable(main main.cpp)
include_directories(${OPENGL_INCLUDE_DIR} ${GLUT_INCLUDE_DIR})
target_link_libraries(main ${OPENGL_gl_LIBRARY} ${OPENGL_glu_LIBRARY} ${GLUT_glut_LIBRARY} KilobotSimulator)
