set(dbow2_ROOT_DIR "${PROJECT_SOURCE_DIR}/orb_slam3/Thirdparty/DBoW2")

set(HDRS_DBOW2
  ${dbow2_ROOT_DIR}/DBoW2/BowVector.h
  ${dbow2_ROOT_DIR}/DBoW2/FORB.h 
  ${dbow2_ROOT_DIR}/DBoW2/FClass.h       
  ${dbow2_ROOT_DIR}/DBoW2/FeatureVector.h
  ${dbow2_ROOT_DIR}/DBoW2/ScoringObject.h   
  ${dbow2_ROOT_DIR}/DBoW2/TemplatedVocabulary.h)

set(SRCS_DBOW2
  ${dbow2_ROOT_DIR}/DBoW2/BowVector.cpp
  ${dbow2_ROOT_DIR}/DBoW2/FORB.cpp      
  ${dbow2_ROOT_DIR}/DBoW2/FeatureVector.cpp
  ${dbow2_ROOT_DIR}/DBoW2/ScoringObject.cpp)

set(HDRS_DUTILS
  ${dbow2_ROOT_DIR}/DUtils/Random.h
  ${dbow2_ROOT_DIR}/DUtils/Timestamp.h)

set(SRCS_DUTILS
  ${dbow2_ROOT_DIR}/DUtils/Random.cpp
  ${dbow2_ROOT_DIR}/DUtils/Timestamp.cpp)

# Ensure the target directories exist before build
file(MAKE_DIRECTORY ${dbow2_ROOT_DIR}/lib)

# Set library output path to avoid source directory overwrite
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# Optionally, set a separate directory for other types of build outputs:
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

include_directories(${OpenCV_INCLUDE_DIRS})

# Add the library as a shared object
add_library(DBoW2 SHARED ${SRCS_DBOW2} ${SRCS_DUTILS})

# Link OpenCV libraries
target_link_libraries(DBoW2 ${OpenCV_LIBS})
