cmake_minimum_required (VERSION 2.8)

set (CMAKE_BUILD_TYPE Release)

project (PCL_HEAD_DIFFUSION)

find_package (PCL REQUIRED)
include_directories (${PCL_INCLUDE_DIRS})

find_package (Eigen REQUIRED)
include_directories (${Eigen_INCLUDE_DIRS})

find_package (CGAL REQUIRED)
include_directories (${CGAL_INCLUDE_DIRS})

find_package (Boost REQUIRED)
include_directories (${Boost_INCLUDE_DIRS})

include_directories (include)
add_definitions (-std=c++0x ${CGAL_CXX_FLAGS_INIT})

add_executable (heat src/main.cpp src/voronoi_diagram.cpp)
target_link_libraries (heat ${Boost_LIBRARIES}  ${CGAL_LIBRARY} ${CGAL_3RD_PARTY_LIBRARIES} ${PCL_LIBRARIES} ${Eigen_LIBRARIES} )
