cmake_minimum_required(VERSION 3.10)
project(Rasterizer)

find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")


include_directories(/usr/include ./include)

add_executable(Rasterizer main.cpp rasterizer.hpp rasterizer.cpp renderer.cpp renderer.hpp global.hpp Triangle.hpp Triangle.cpp Texture.hpp Shader.hpp OBJ_Loader.h)
target_link_libraries(Rasterizer ${OpenCV_LIBRARIES} restbed)
#target_compile_options(Rasterizer PUBLIC -Wall -Wextra -pedantic)