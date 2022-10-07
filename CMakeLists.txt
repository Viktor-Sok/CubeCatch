cmake_minimum_required(VERSION 3.10)
project (pose_move CXX)
set(CMAKE_CXX_STANDARD 11)
set (CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bin) # .exe output folder
#set(CMAKE_PREFIX_PATH  "C:/Cpp_Projects/Protobuf_GCC_Eclipse/protobuf_installed") # path where to find a package
set(CMAKE_PREFIX_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib/protobuf_installed)
# library for parsing JSON
find_package(nlohmann_json 3.2.0 REQUIRED PATHS "C:/libs/json-develop_build")
# Google Protobuf
find_package(Protobuf REQUIRED)
# Eigen library for linear algebra
find_package(Eigen3 3.4 REQUIRED)
# OpenCV for computer vision
set(OpenCV_DIR "C:/libs/OpenCV455")
find_package(OpenCV REQUIRED)
#include_directories(${Protobuf_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR} ${Protobuf_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS egm.proto)
file(GLOB_RECURSE sources ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp)
add_executable(
${PROJECT_NAME} ${sources}
${PROTO_SRCS} ${PROTO_HDRS}
)
target_link_libraries(${PROJECT_NAME} ${Protobuf_LIBRARIES} Eigen3::Eigen nlohmann_json::nlohmann_json ${OpenCV_LIBS})
target_include_directories(${PROJECT_NAME} PUBLIC ${PROJECT_SOURCE_DIR}/include)
if(WIN32)
target_link_libraries(${PROJECT_NAME} wsock32 ws2_32)
endif()

