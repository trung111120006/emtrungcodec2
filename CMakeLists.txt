cmake_minimum_required(VERSION 3.0)

set(CMAKE_CXX_STANDARD 14)

project(VR_streaming)

include_directories(${PROJECT_SOURCE_DIR})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})

add_executable(encode
    encode.cpp
    
    pointcloud.cpp
    
    miniply.cpp
    
    octree.cpp

)
target_link_libraries(encode)

add_executable(decode
   decode.cpp

    pointcloud.cpp
    
    miniply.cpp

    octree.cpp
)
target_link_libraries(decode pthread)