cmake_minimum_required(VERSION 3.6)
project(Sumi)

set(external_source_dir ${PROJECT_SOURCE_DIR}/external/src)
set(source_dir ${PROJECT_SOURCE_DIR}/src)
set(include_dir ${PROJECT_SOURCE_DIR}/include)
include_directories(${include_dir})

set(external_libs "")
set(external_srcs "")

#glfw
add_subdirectory(${external_source_dir}/glfw-3.3.9)
list(APPEND external_libs glfw)

#glad
include_directories(${external_source_dir}/glad/include)
list(APPEND external_srcs ${external_source_dir}/glad/src/glad.c)

# ImGui
include_directories(${include_dir}/imgui)
include_directories(${external_source_dir}/ImGuiFileDialog)

#stb
include_directories(${include_dir}/stb)

#main srcs
file(GLOB_RECURSE srcs ${source_dir}/*.cpp)
include_directories(${include_dir}/cpu)

#exe
add_executable(${PROJECT_NAME} ${srcs} ${external_srcs})

target_link_libraries(${PROJECT_NAME} ${external_libs})
# target_compile_options(${PROJECT_NAME} PRIVATE "-Wall")