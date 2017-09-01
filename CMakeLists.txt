project(Transformation)
cmake_minimum_required(VERSION 3.0)

include(FindEigen3.cmake)

find_package( Eigen3 3.3 REQUIRED NO_MODULE )

set( src
	src/Transformation.h
	src/Transformation.cpp
)
add_library(Transformation ${src})
target_link_libraries(Transformation Eigen3::Eigen)

add_executable(exp_rigid src/exp_rigid.cpp)
target_link_libraries(exp_rigid Transformation)
	
add_executable(exp_similarity src/exp_similarity.cpp)
target_link_libraries(exp_similarity Transformation)
