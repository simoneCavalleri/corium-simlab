cmake_minimum_required(VERSION 3.20)
project(corium_simlab LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Option to specify local Corium path
set(CORIUM_LOCAL_PATH "" CACHE PATH "Path to local Corium repository")

include(FetchContent)

# 1. Fetch / Include Corium
if(CORIUM_LOCAL_PATH AND EXISTS ${CORIUM_LOCAL_PATH})
    message(STATUS "Using local Corium path: ${CORIUM_LOCAL_PATH}")
    add_subdirectory(${CORIUM_LOCAL_PATH} corium_build EXCLUDE_FROM_ALL)
else()
    message(STATUS "Fetching Corium from GitHub...")
    FetchContent_Declare(
        corium
        GIT_REPOSITORY https://github.com/simoneCavalleri/corium.git
        GIT_TAG main
    )
    FetchContent_MakeAvailable(corium)
endif()

# 2. Fetch GLFW for Windowing
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG 3.4
)
FetchContent_MakeAvailable(glfw)

# 3. Fetch wgpu-native prebuilt C binaries & headers
FetchContent_Declare(
    wgpu_native
    URL https://github.com/gfx-rs/wgpu-native/releases/download/v0.19.4.1/wgpu-linux-x86_64-release.zip
)
FetchContent_MakeAvailable(wgpu_native)

# Locate wgpu_native library
find_library(WGPU_NATIVE_LIB NAMES wgpu_native libwgpu_native.so PATHS ${wgpu_native_SOURCE_DIR} NO_DEFAULT_PATH)

# Engine Core Static Library (separating headers from .cpp implementations)
add_library(corium_sim_core STATIC
    src/renderer/Camera.cpp
    src/renderer/Mesh.cpp
    src/renderer/Texture.cpp
    src/renderer/WebGpuBackend.cpp
    src/services/TelemetryService.cpp
    src/App.cpp
)

target_include_directories(corium_sim_core PUBLIC include ${wgpu_native_SOURCE_DIR})
target_link_libraries(corium_sim_core PUBLIC corium glfw ${WGPU_NATIVE_LIB})

# Main Application Executable (corium-simlab)
add_executable(corium-simlab src/main.cpp)
target_link_libraries(corium-simlab PRIVATE corium_sim_core)

# Ensure libwgpu_native.so is copied next to the final executable
add_custom_command(TARGET corium-simlab POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "${wgpu_native_SOURCE_DIR}/libwgpu_native.so"
    "$<TARGET_FILE_DIR:corium-simlab>"
)
