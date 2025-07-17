cmake_minimum_required(VERSION 3.20.0)
project(visual_slam VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall -O3")
## Code optimization for the highest level at the release build time.
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -g -O0") 
## No code optimization at the debug build time
# set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -g -O0 -fsanitize=address")
# set(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} -fsanitize=address")
# set(CMAKE_SHARED_LINKER_FLAGS_DEBUG "${CMAKE_SHARED_LINKER_FLAGS_DEBUG} -fsanitize=address")
## No code optimization and check the memory  at the debug build time

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
## For the productivity of the development.
# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
## It is not necessary to set the output directory for the library,
## because of source directory pollution. (maybe remove it later.)

# RPATH (Runtime Library Search Path) related options
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE) 

find_package(yaml-cpp REQUIRED  PATHS "${PROJECT_SOURCE_DIR}/thirdparty/yaml-cpp")
find_package(Eigen3 REQUIRED PATHS "${PROJECT_SOURCE_DIR}/thirdparty/eigen")
find_package(OpenCV REQUIRED PATHS "${PROJECT_SOURCE_DIR}/thirdparty/opencv")
find_package(g2o REQUIRED PATHS "${PROJECT_SOURCE_DIR}/thirdparty/g2o")
find_package(DBoW2 REQUIRED PATHS "${PROJECT_SOURCE_DIR}/thirdparty/dbow2")
find_package(Pangolin REQUIRED PATHS "${PROJECT_SOURCE_DIR}/thirdparty/pangolin")
get_filename_component(DBOW2_LIBRARY_DIR ${DBoW2_LIBRARIES} DIRECTORY)
list(APPEND CMAKE_BUILD_RPATH "${DBOW2_LIBRARY_DIR}")

add_library(slam SHARED 
    src/misc.cpp
    src/settings.cpp
    src/camera.cpp
    src/geometry.cpp
    src/optimization.cpp
    
    src/map_point.cpp
    src/frame.cpp
    src/map.cpp
    src/viewer3d.cpp
   
    src/quadtree.cpp
    src/matcher.cpp
    src/extractor.cpp
    src/tracker.cpp    
    src/bow.cpp
    src/vslam.cpp
    src/system.cpp
)
target_link_libraries(slam PUBLIC 
    yaml-cpp::yaml-cpp
    Eigen3::Eigen
    ${OpenCV_LIBS}
    g2o::core
    g2o::stuff
    g2o::types_sba
    g2o::types_slam3d
    g2o::solver_csparse
    g2o::opengl_helper
    ${DBoW2_LIBRARIES}
    ${Pangolin_LIBRARIES}
)
target_include_directories(slam PUBLIC
    ${PROJECT_SOURCE_DIR}/include
    # ${OpenCV_INCLUDE_DIRS}
    # ${Eigen3_INCLUDE_DIRS}
    ${DBoW2_INCLUDE_DIRS}
    ) 

add_executable(main examples/main.cpp)
target_link_libraries(main slam)
add_executable(make_vocabulary examples/make_vocabulary.cpp)
target_link_libraries(make_vocabulary slam)
add_executable(test_vocabulary examples/test_vocabulary.cpp)
target_link_libraries(test_vocabulary slam)

add_executable(pose_estimation_test 
    examples/pose_estimation_test.cpp
    src/misc.cpp
    src/settings.cpp
    src/extractor.cpp
    src/quadtree.cpp

    )
# add_library(misc SHARED src/misc.cpp)
target_include_directories(pose_estimation_test PRIVATE 
    ${PROJECT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    )
target_link_libraries(pose_estimation_test PRIVATE 
    ${OpenCV_LIBS}
    yaml-cpp::yaml-cpp
)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_definitions(slam PUBLIC DEBUG_MODE)
    # Transfer the DEBUG_MODE flag to the target files.
    message("yaml-cpp ${yaml-cpp_VERSION} found")
    message("Eigen3 ${Eigen3_VERSION} found")
    message("OpenCV ${OpenCV_VERSION} found")
    message("g2o ${g2o_VERSION} found")
    message("DBoW2 ${DBoW2_VERSION} found")
    message("Pangolin ${Pangolin_VERSION} found")
endif()


# set(CPACK_PROJECT_NAME ${PROJECT_NAME})
# set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
# include(CPack)
