cmake_minimum_required(VERSION 3.20.0)
project(visual_slam VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_definitions(-DDEBUG_MODE)
endif()

# set(CPACK_PROJECT_NAME ${PROJECT_NAME})
# set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
# include(CPack)

include_directories(include)
add_library(slam STATIC 
    src/vslam.cpp
    src/settings.cpp
)

find_package(yaml-cpp 0.8.0 EXACT REQUIRED PATHS "thirdparty/yaml-cpp")
if(yaml-cpp_FOUND)
    message("yaml-cpp ${yaml-cpp_VERSION} found")
    target_link_libraries(slam PUBLIC yaml-cpp::yaml-cpp)
else()
    message("yaml-cpp not found")
endif()

find_package(Eigen3 REQUIRED PATHS "thirdparty/eigen")
if(Eigen3_FOUND)
    message("Eigen3 ${Eigen3_VERSION} found")
    target_link_libraries(slam PUBLIC Eigen3::Eigen)
else()
    message("Eigen3 not found")
endif()

find_package(OpenCV 4.11.0 EXACT REQUIRED PATHS "thirdparty/opencv")
if(OpenCV_FOUND)
    message("OpenCV ${OpenCV_VERSION} found")
    target_link_libraries(slam PUBLIC ${OpenCV_LIBS})
else()
    message("OpenCV not found")
endif()

find_package(g2o 1.0.0 EXACT REQUIRED PATHS "thirdparty/g2o")
if(g2o_FOUND)
    message("g2o ${g2o_VERSION} found")
    target_link_libraries(slam PUBLIC g2o::core)
endif()

find_package(Pangolin 0.9.2 EXACT REQUIRED PATHS "thirdparty/pangolin")
if(Pangolin_FOUND)
    message("Pangolin ${Pangolin_VERSION} found")
    target_link_libraries(slam PUBLIC ${Pangolin_LIBRARY})
endif()


add_executable(main examples/main.cpp)
target_link_libraries(main slam)
