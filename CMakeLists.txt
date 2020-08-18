cmake_minimum_required (VERSION 3.5)
project (pcl_lbo LANGUAGES CXX)

set (CMAKE_CXX_STANDARD 14)

find_package (PCL REQUIRED)
find_package (Eigen3 REQUIRED)
find_package (CGAL REQUIRED)
find_package (Boost REQUIRED COMPONENTS program_options filesystem)
find_package (SuiteSparse REQUIRED COMPONENTS CHOLMOD)

add_library (PCL::PCL INTERFACE IMPORTED)
target_link_libraries (PCL::PCL INTERFACE ${PCL_LIBRARIES})
target_include_directories (PCL::PCL INTERFACE ${PCL_INCLUDE_DIRS})

add_library (SuiteSparse::Cholmod INTERFACE IMPORTED)
target_link_libraries (SuiteSparse::Cholmod INTERFACE ${SuiteSparse_CHOLMOD_LIBRARY_RELEASE})
target_include_directories (SuiteSparse::Cholmod INTERFACE ${SuiteSparse_INCLUDE_DIRS})

message("VERGA ${SuiteSparse_CHOLMOD_LIBRARY}")

add_library (pclbo
    "src/meshlbo.cpp"
    "src/meshgeoheat.cpp"
    "src/voronoi_diagram.cpp")

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


add_executable (geoheat EXCLUDE_FROM_ALL src/geoheat_demo.cpp)
target_link_libraries (geoheat 
  PRIVATE
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
