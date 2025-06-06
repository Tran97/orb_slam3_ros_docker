set(g2o_SOURCE_DIR "${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/g2o")

SET(g2o_C_FLAGS)
SET(g2o_CXX_FLAGS)

SET(G2O_LIB_TYPE SHARED)

# There seems to be an issue with MSVC8
# see http://eigen.tuxfamily.org/bz/show_bug.cgi?id=83
if(MSVC90)
  add_definitions(-DEIGEN_DONT_ALIGN_STATICALLY=1)
  message(STATUS "Disabling memory alignment for MSVC8")
endif(MSVC90)

# Set the output directory for the build executables and libraries
# Ensure the library output path is properly set to a writable directory
# Use the build directory instead of source directory to avoid overwriting
set(g2o_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Set output directory for both libraries and archives
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${g2o_LIBRARY_OUTPUT_DIRECTORY})
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${g2o_LIBRARY_OUTPUT_DIRECTORY})

# Create the output directories if they don't exist
file(MAKE_DIRECTORY ${g2o_LIBRARY_OUTPUT_DIRECTORY})

# Set search directory for looking for custom CMake modules
LIST(APPEND CMAKE_MODULE_PATH ${g2o_SOURCE_DIR}/cmake_modules)

# Detect OS and define macros appropriately
IF(UNIX)
  ADD_DEFINITIONS(-DUNIX)
  MESSAGE(STATUS "Compiling on Unix")
ENDIF(UNIX)

# OPENMP is experimental, disable it by default
FIND_PACKAGE(OpenMP)
SET(G2O_USE_OPENMP OFF CACHE BOOL "Build g2o with OpenMP support (EXPERIMENTAL)")
IF(OPENMP_FOUND AND G2O_USE_OPENMP)
  SET (G2O_OPENMP 1)
  SET(g2o_C_FLAGS "${g2o_C_FLAGS} ${OpenMP_C_FLAGS}")
  SET(g2o_CXX_FLAGS "${g2o_CXX_FLAGS} -DEIGEN_DONT_PARALLELIZE ${OpenMP_CXX_FLAGS}")
  MESSAGE(STATUS "Compiling with OpenMP support")
ENDIF(OPENMP_FOUND AND G2O_USE_OPENMP)

# Enable warnings
SET(g2o_C_FLAGS "${g2o_C_FLAGS} -Wall -W")
SET(g2o_CXX_FLAGS "${g2o_CXX_FLAGS} -Wall -W")

# Specify compiler flags
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${g2o_CXX_FLAGS}")
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${g2o_C_FLAGS}")

# Generate config.h
SET(G2O_CXX_COMPILER "${CMAKE_CXX_COMPILER_ID} ${CMAKE_CXX_COMPILER}")
configure_file(${g2o_SOURCE_DIR}/config.h.in ${g2o_SOURCE_DIR}/config.h)

# Set up include directories
INCLUDE_DIRECTORIES(
  ${g2o_SOURCE_DIR}/g2o/core
  ${g2o_SOURCE_DIR}/g2o/types
  ${g2o_SOURCE_DIR}/g2o/stuff 
  ${G2O_EIGEN3_INCLUDE}
)

# Add the g2o library
ADD_LIBRARY(g2o ${G2O_LIB_TYPE}
  #types
  ${g2o_SOURCE_DIR}/g2o/types/types_sba.h
  ${g2o_SOURCE_DIR}/g2o/types/types_six_dof_expmap.h
  ${g2o_SOURCE_DIR}/g2o/types/types_sba.cpp
  ${g2o_SOURCE_DIR}/g2o/types/types_six_dof_expmap.cpp
  ${g2o_SOURCE_DIR}/g2o/types/types_seven_dof_expmap.cpp
  ${g2o_SOURCE_DIR}/g2o/types/types_seven_dof_expmap.h
  ${g2o_SOURCE_DIR}/g2o/types/se3quat.h
  ${g2o_SOURCE_DIR}/g2o/types/se3_ops.h
  ${g2o_SOURCE_DIR}/g2o/types/se3_ops.hpp
  #core
  ${g2o_SOURCE_DIR}/g2o/core/base_edge.h
  ${g2o_SOURCE_DIR}/g2o/core/base_binary_edge.h
  ${g2o_SOURCE_DIR}/g2o/core/hyper_graph_action.cpp
  ${g2o_SOURCE_DIR}/g2o/core/base_binary_edge.hpp
  ${g2o_SOURCE_DIR}/g2o/core/hyper_graph_action.h
  ${g2o_SOURCE_DIR}/g2o/core/base_multi_edge.h           
  ${g2o_SOURCE_DIR}/g2o/core/hyper_graph.cpp
  ${g2o_SOURCE_DIR}/g2o/core/base_multi_edge.hpp         
  ${g2o_SOURCE_DIR}/g2o/core/hyper_graph.h
  ${g2o_SOURCE_DIR}/g2o/core/base_unary_edge.h          
  ${g2o_SOURCE_DIR}/g2o/core/linear_solver.h
  ${g2o_SOURCE_DIR}/g2o/core/base_unary_edge.hpp         
  ${g2o_SOURCE_DIR}/g2o/core/marginal_covariance_cholesky.cpp
  ${g2o_SOURCE_DIR}/g2o/core/base_vertex.h               
  ${g2o_SOURCE_DIR}/g2o/core/marginal_covariance_cholesky.h
  ${g2o_SOURCE_DIR}/g2o/core/base_vertex.hpp             
  ${g2o_SOURCE_DIR}/g2o/core/matrix_structure.cpp
  ${g2o_SOURCE_DIR}/g2o/core/batch_stats.cpp             
  ${g2o_SOURCE_DIR}/g2o/core/matrix_structure.h
  ${g2o_SOURCE_DIR}/g2o/core/batch_stats.h               
  ${g2o_SOURCE_DIR}/g2o/core/openmp_mutex.h
  ${g2o_SOURCE_DIR}/g2o/core/block_solver.h              
  ${g2o_SOURCE_DIR}/g2o/core/block_solver.hpp            
  ${g2o_SOURCE_DIR}/g2o/core/parameter.cpp               
  ${g2o_SOURCE_DIR}/g2o/core/parameter.h                 
  ${g2o_SOURCE_DIR}/g2o/core/cache.cpp                   
  ${g2o_SOURCE_DIR}/g2o/core/cache.h
  ${g2o_SOURCE_DIR}/g2o/core/optimizable_graph.cpp       
  ${g2o_SOURCE_DIR}/g2o/core/optimizable_graph.h         
  ${g2o_SOURCE_DIR}/g2o/core/solver.cpp                  
  ${g2o_SOURCE_DIR}/g2o/core/solver.h
  ${g2o_SOURCE_DIR}/g2o/core/creators.h                 
  ${g2o_SOURCE_DIR}/g2o/core/optimization_algorithm_factory.cpp
  ${g2o_SOURCE_DIR}/g2o/core/estimate_propagator.cpp     
  ${g2o_SOURCE_DIR}/g2o/core/optimization_algorithm_factory.h
  ${g2o_SOURCE_DIR}/g2o/core/estimate_propagator.h       
  ${g2o_SOURCE_DIR}/g2o/core/factory.cpp                 
  ${g2o_SOURCE_DIR}/g2o/core/optimization_algorithm_property.h
  ${g2o_SOURCE_DIR}/g2o/core/factory.h                   
  ${g2o_SOURCE_DIR}/g2o/core/sparse_block_matrix.h
  ${g2o_SOURCE_DIR}/g2o/core/sparse_optimizer.cpp  
  ${g2o_SOURCE_DIR}/g2o/core/sparse_block_matrix.hpp
  ${g2o_SOURCE_DIR}/g2o/core/sparse_optimizer.h
  ${g2o_SOURCE_DIR}/g2o/core/hyper_dijkstra.cpp 
  ${g2o_SOURCE_DIR}/g2o/core/hyper_dijkstra.h
  ${g2o_SOURCE_DIR}/g2o/core/parameter_container.cpp     
  ${g2o_SOURCE_DIR}/g2o/core/parameter_container.h
  ${g2o_SOURCE_DIR}/g2o/core/optimization_algorithm.cpp 
  ${g2o_SOURCE_DIR}/g2o/core/optimization_algorithm.h
  ${g2o_SOURCE_DIR}/g2o/core/optimization_algorithm_with_hessian.cpp 
  ${g2o_SOURCE_DIR}/g2o/core/optimization_algorithm_with_hessian.h
  ${g2o_SOURCE_DIR}/g2o/core/optimization_algorithm_levenberg.cpp 
  ${g2o_SOURCE_DIR}/g2o/core/optimization_algorithm_levenberg.h
  ${g2o_SOURCE_DIR}/g2o/core/optimization_algorithm_gauss_newton.cpp 
  ${g2o_SOURCE_DIR}/g2o/core/optimization_algorithm_gauss_newton.h
  ${g2o_SOURCE_DIR}/g2o/core/jacobian_workspace.cpp 
  ${g2o_SOURCE_DIR}/g2o/core/jacobian_workspace.h
  ${g2o_SOURCE_DIR}/g2o/core/robust_kernel.cpp 
  ${g2o_SOURCE_DIR}/g2o/core/robust_kernel.h
  ${g2o_SOURCE_DIR}/g2o/core/robust_kernel_factory.cpp
  ${g2o_SOURCE_DIR}/g2o/core/robust_kernel_factory.h
  ${g2o_SOURCE_DIR}/g2o/core/robust_kernel_impl.cpp 
  ${g2o_SOURCE_DIR}/g2o/core/robust_kernel_impl.h
#stuff
  ${g2o_SOURCE_DIR}/g2o/stuff/string_tools.h
  ${g2o_SOURCE_DIR}/g2o/stuff/color_macros.h 
  ${g2o_SOURCE_DIR}/g2o/stuff/macros.h
  ${g2o_SOURCE_DIR}/g2o/stuff/timeutil.cpp
  ${g2o_SOURCE_DIR}/g2o/stuff/misc.h
  ${g2o_SOURCE_DIR}/g2o/stuff/timeutil.h
  ${g2o_SOURCE_DIR}/g2o/stuff/os_specific.c    
  ${g2o_SOURCE_DIR}/g2o/stuff/os_specific.h
  ${g2o_SOURCE_DIR}/g2o/stuff/string_tools.cpp
  ${g2o_SOURCE_DIR}/g2o/stuff/property.cpp       
  ${g2o_SOURCE_DIR}/g2o/stuff/property.h       
)
