cmake_minimum_required (VERSION 2.8)


project (PCL_HEAD_DIFFUSION)

find_package (PCL REQUIRED)
include_directories (${PCL_INCLUDE_DIRS})

find_package (Eigen3 REQUIRED)
include_directories (${Eigen3_INCLUDE_DIRS})

find_package (CGAL REQUIRED)
include_directories (${CGAL_INCLUDE_DIRS})

find_package (Boost REQUIRED)
include_directories (${Boost_INCLUDE_DIRS})

include_directories (include)

add_definitions (-std=c++0x)

add_executable (heat src/main.cpp src/voronoi_diagram.cpp)
target_link_libraries (heat ${Boost_LIBRARIES} gmp CGAL ${PCL_LIBRARIES} ${Eigen3_LIBRARIES} )
