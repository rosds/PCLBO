cmake_minimum_required (VERSION 2.8)

set (CMAKE_BUILD_TYPE Release)

project (pcl_lbo)

##############################################################################
# Required packages                                                          #
##############################################################################

# To handle the point clouds, compute normals and KdTree
find_package (PCL REQUIRED)
include_directories (${PCL_INCLUDE_DIRS})
link_directories (${PCL_LIBRARY_DIRS})

# Depending on the installation the package name might be Eigen3
find_package (Eigen3 REQUIRED)
include_directories (${Eigen3_INCLUDE_DIRS})

# CGAL Library used only to compute the Voronoi diagram
find_package (CGAL REQUIRED)
include_directories (${CGAL_INCLUDE_DIRS})

find_package (Boost COMPONENTS program_options filesystem REQUIRED)
include_directories (${Boost_INCLUDE_DIRS})

##############################################################################
# C++11 support required                                                     #
##############################################################################

include_directories (include)
add_definitions (-std=c++0x ${CGAL_CXX_FLAGS_INIT})

##############################################################################
# Compile the library                                                        #
##############################################################################

set (lbo_src 
    "src/meshlbo.cpp"
    "src/meshgeoheat.cpp")
add_library (pclbo SHARED ${lbo_src})

##############################################################################
# Compile a demo                                                             #
##############################################################################

add_executable (visualize_eigenfunctions src/eigenfunctions_demo.cpp)
target_link_libraries (visualize_eigenfunctions 
    ${Boost_LIBRARIES}  
    ${CGAL_LIBRARY} 
    ${CGAL_3RD_PARTY_LIBRARIES} 
    ${PCL_LIBRARIES} 
    ${Eigen3_LIBRARIES})

add_executable (heat src/heat_demo.cpp)
target_link_libraries (heat 
    ${Boost_LIBRARIES}  
    ${CGAL_LIBRARY} 
    ${CGAL_3RD_PARTY_LIBRARIES} 
    ${PCL_LIBRARIES} 
    ${Eigen3_LIBRARIES})

add_executable (geoheat src/geoheat_demo.cpp)
target_link_libraries (geoheat 
    ${Boost_LIBRARIES}  
    ${CGAL_LIBRARY} 
    ${CGAL_3RD_PARTY_LIBRARIES} 
    ${PCL_LIBRARIES} 
    ${Eigen3_LIBRARIES})


include_directories ("/usr/include/suitesparse")
add_executable (mesh_geoheat src/mesh_geoheat_demo.cpp)
target_link_libraries (mesh_geoheat 
    pclbo
    cholmod
    ${Boost_LIBRARIES}  
    ${PCL_LIBRARIES} 
    ${Eigen3_LIBRARIES})
