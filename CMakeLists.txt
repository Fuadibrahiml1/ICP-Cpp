cmake_minimum_required(VERSION 3.12)
project(KISS_SLAM)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include_directories(
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
        "${CMAKE_CURRENT_SOURCE_DIR}/config"
        "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/eigen"
        "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/Sophus-main/sophus"
        "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/pcl-master/io/src"
)

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_subdirectory(thirdparty/oneTBB-master)

add_executable(KISS-SLAM
        "${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/mainPipeline/pipeline.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/dataLoader.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/src/csvFileCreator.cpp"
        src/csvToPointCloud.cpp
        include/csvToPointCLoud.hpp
        core/visualizerPoint.cpp
        core/visualizerPoint.hpp
        ramka/Registration.cpp
        ramka/Registration.hpp
        src/dataLogger.cpp

)

target_link_libraries(KISS-SLAM PRIVATE
        ${PCL_LIBRARIES}
        TBB::tbb
)