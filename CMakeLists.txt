cmake_minimum_required(VERSION 3.14)
project(PhysicsEngine CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# --- System dependencies ---
find_package(SFML 3 REQUIRED COMPONENTS Graphics Window System)

# OpenGL legacy (fixed-function pipeline + GLU)
set(OpenGL_GL_PREFERENCE GLVND)
find_package(OpenGL REQUIRED COMPONENTS OpenGL)
find_package(GLUT REQUIRED)

# Some distros split GLU out of the OpenGL target; locate it explicitly.
find_library(GLU_LIBRARY NAMES GLU REQUIRED)

# --- Fetch ImGui and ImGui-SFML (SFML 3 compatible) ---
include(FetchContent)

FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG        v1.91.5
)
FetchContent_MakeAvailable(imgui)

set(IMGUI_DIR ${imgui_SOURCE_DIR} CACHE PATH "" FORCE)
set(IMGUI_SFML_FIND_SFML OFF CACHE BOOL "" FORCE)
set(IMGUI_SFML_IMGUI_DEMO OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
    imgui-sfml
    GIT_REPOSITORY https://github.com/SFML/imgui-sfml.git
    GIT_TAG        v3.0
)
FetchContent_MakeAvailable(imgui-sfml)

# --- Executable ---
add_executable(physics_engine
    main.cpp
    physics.cpp
)

target_link_libraries(physics_engine PRIVATE
    ImGui-SFML::ImGui-SFML
    SFML::Graphics
    SFML::Window
    SFML::System
    OpenGL::GL
    ${GLU_LIBRARY}
    GLUT::GLUT
)
