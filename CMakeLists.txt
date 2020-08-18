cmake_minimum_required (VERSION 3.5)
project (pcl_lbo LANGUAGES CXX)

set (CMAKE_CXX_STANDARD 14)
list (APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package (PCL REQUIRED)
find_package (Eigen3 REQUIRED)
find_package (CGAL REQUIRED)
find_package (Boost REQUIRED COMPONENTS program_options filesystem)
find_package (SuiteSparse REQUIRED)

add_library (PCL::PCL INTERFACE IMPORTED)
target_link_libraries (PCL::PCL INTERFACE ${PCL_LIBRARIES})
target_include_directories (PCL::PCL INTERFACE ${PCL_INCLUDE_DIRS})

add_library (SuiteSparse::Cholmod INTERFACE IMPORTED)
target_link_libraries (SuiteSparse::Cholmod INTERFACE ${SuiteSparse_CHOLMOD_LIBRARY})
target_include_directories (SuiteSparse::Cholmod INTERFACE ${SuiteSparse_CHOLMOD_INCLUDE_DIR})

add_library (pclbo
    "include/heat/heat.h"
    "include/heat/impl/heat.hpp"
    "include/pclbo/geodesics/meshgeoheat.h"
    "include/pclbo/meshlbo.h"
    "include/pclbo/pclbo.h"
    "include/pclbo/utils.h"
    "include/pclbo/voronoi_diagram.h"
    "include/pclbo/impl/pclbo.hpp"
    "src/meshlbo.cpp"
    "src/meshgeoheat.cpp")

target_include_directories (pclbo PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_link_libraries (pclbo
    PUBLIC
        PCL::PCL
    PRIVATE
        CGAL::CGAL
        Eigen3::Eigen
        SuiteSparse::Cholmod)

install (TARGETS pclbo DESTINATION lib)
install (DIRECTORY include/pclbo DESTINATION include)

##############################################################################
# Compile a demo                                                             #
##############################################################################

add_executable (visualize_eigenfunctions EXCLUDE_FROM_ALL src/eigenfunctions_demo.cpp)
target_link_libraries (visualize_eigenfunctions 
  PRIVATE
    pclbo
    Boost::program_options
    Boost::filesystem)

add_executable (heat EXCLUDE_FROM_ALL src/heat_demo.cpp)
target_link_libraries (heat 
  PRIVATE
    pclbo)


add_executable (geoheat src/geoheat_demo.cpp)
target_link_libraries (geoheat 
    pclbo
    ${CGAL_LIBRARY} 
    ${CGAL_3RD_PARTY_LIBRARIES} 
    ${PCL_LIBRARIES} 
    ${Eigen3_LIBRARIES})


add_executable (mesh_geoheat EXCLUDE_FROM_ALL src/mesh_geoheat_demo.cpp)
target_link_libraries (mesh_geoheat 
  PRIVATE
    pclbo
    Boost::program_options
    Boost::filesystem)
