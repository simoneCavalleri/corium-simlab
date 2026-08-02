cmake_minimum_required(VERSION 3.20)
project(corium_simlab LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Option to specify local Corium path
set(CORIUM_LOCAL_PATH "" CACHE PATH "Path to local Corium repository")

include(FetchContent)

# 1. Fetch / Include Corium Engine Core
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

# 2. Fetch GLFW for Windowing & Native Display Surface
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

# Engine Core Static Library (corium_sim_core)
add_library(corium_sim_core STATIC
    src/renderer/Camera.cpp
    src/renderer/Mesh.cpp
    src/renderer/MeshLoader.cpp
    src/renderer/SensorCamera.cpp
    src/renderer/Texture.cpp
    src/renderer/WebGpuBackend.cpp
    src/renderer/WebGpuSurface.cpp
    src/scene/SimScene.cpp
    src/scene/SceneBuilder.cpp
    src/physics/PhysicsEngine.cpp
    src/kinematics/JointKinematics.cpp
    src/App.cpp
)

target_include_directories(corium_sim_core PUBLIC include ${wgpu_native_SOURCE_DIR})
target_compile_definitions(corium_sim_core PUBLIC CORIUM_SIMLAB_ROOT_DIR="${CMAKE_SOURCE_DIR}")
target_link_libraries(corium_sim_core PUBLIC corium glfw ${WGPU_NATIVE_LIB})

# 4. Optional Python C++ Extension Binding (pybind11)
option(BUILD_PYTHON_BINDINGS "Build Python C++ Extension module" ON)

if(BUILD_PYTHON_BINDINGS)
    message(STATUS "Fetching pybind11 for Python environment bindings...")
    FetchContent_Declare(
        pybind11
        GIT_REPOSITORY https://github.com/pybind/pybind11.git
        GIT_TAG v2.12.0
    )
    FetchContent_MakeAvailable(pybind11)

    pybind11_add_module(corium_sim_py src/python/bindings.cpp)
    target_link_libraries(corium_sim_py PRIVATE corium_sim_core)

    # Ensure libwgpu_native.so is copied next to Python extension module
    add_custom_command(TARGET corium_sim_py POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${wgpu_native_SOURCE_DIR}/libwgpu_native.so"
        "$<TARGET_FILE_DIR:corium_sim_py>"
    )
endif()

# 5. Build C++ Samples Directory Targets
option(BUILD_SAMPLES "Build C++ Samples" ON)

if(BUILD_SAMPLES)
    add_subdirectory(samples/cpp)
endif()
