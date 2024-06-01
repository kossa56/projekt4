cmake_minimum_required(VERSION 3.15)
project(Projekt3)
set(CMAKE_CXX_STANDARD 17)

include(cmake/CPM.cmake)

add_subdirectory(matplotplusplus)
add_subdirectory(SDL)
add_subdirectory(SDL2_gfx)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}")

CPMAddPackage(
  NAME Eigen
  VERSION 3.4.0
  URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
  # Eigen's CMakelists are not intended for library use
  DOWNLOAD_ONLY YES 
)

if(Eigen_ADDED)
  add_library(Eigen INTERFACE IMPORTED)
  target_include_directories(Eigen INTERFACE ${Eigen_SOURCE_DIR})
endif()

set(PLANAR_QUADROTOR_SOURCES PlanarQuadrotor/simulate.cpp PlanarQuadrotor/planar_quadrotor.cpp PlanarQuadrotor/planar_quadrotor_visualizer.cpp PlanarQuadrotor/lqr.cpp)
add_executable(planar_quadrotor ${PLANAR_QUADROTOR_SOURCES})
if(TARGET SDL2::SDL2main)
    # It has an implicit dependency on SDL2 functions, so it MUST be added before SDL2::SDL2 (or SDL2::SDL2-static)
    target_link_libraries(planar_quadrotor SDL2::SDL2main)
endif()
target_link_libraries(planar_quadrotor SDL2::SDL2 SDL2_gfx Eigen matplot)
target_include_directories(planar_quadrotor PRIVATE ${PROJECT_SOURCE_DIR})
