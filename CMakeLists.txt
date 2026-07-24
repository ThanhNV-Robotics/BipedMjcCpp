cmake_minimum_required(VERSION 3.16)
project(BipedMjcCpp LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ------------------------------------------------------------------
# MuJoCo (prebuilt binary release vendored under third_party/mujoco)
# ------------------------------------------------------------------
set(MUJOCO_DIR ${CMAKE_SOURCE_DIR}/third_party/mujoco)

add_library(mujoco::mujoco SHARED IMPORTED)
set_target_properties(mujoco::mujoco PROPERTIES
  IMPORTED_LOCATION ${MUJOCO_DIR}/lib/libmujoco.so
  INTERFACE_INCLUDE_DIRECTORIES ${MUJOCO_DIR}/include
)

# ------------------------------------------------------------------
# Dependencies needed to build MuJoCo's own "simulate" GUI from source
# ------------------------------------------------------------------
find_package(glfw3 3.3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(Threads REQUIRED)

# lodepng: vendored, required by simulate.cc for screenshot capture
add_library(lodepng STATIC third_party/lodepng/lodepng.cpp)
target_include_directories(lodepng PUBLIC third_party/lodepng)

# MuJoCo's simulate UI sources, vendored under third_party/mujoco/simulate
set(MUJOCO_SIMULATE_DIR ${MUJOCO_DIR}/simulate)
add_library(mujoco_simulate STATIC
  ${MUJOCO_SIMULATE_DIR}/simulate.cc
  ${MUJOCO_SIMULATE_DIR}/glfw_adapter.cc
  ${MUJOCO_SIMULATE_DIR}/glfw_dispatch.cc
  ${MUJOCO_SIMULATE_DIR}/platform_ui_adapter.cc
)
target_include_directories(mujoco_simulate PUBLIC ${MUJOCO_SIMULATE_DIR})
target_link_libraries(mujoco_simulate PUBLIC
  mujoco::mujoco
  glfw
  OpenGL::GL
  Threads::Threads
  lodepng
)

# ------------------------------------------------------------------
# core: the project's own code (algorithm/common/math/sim_interface, as
# it grows). Every demo/test executable links against this instead of
# repeating source lists, mirroring OpenLoong-Dyn-Control's structure.
# GLOB on a folder that doesn't exist yet (algorithm/, common/, math/)
# just yields nothing, so this stays a no-op until those show up.
# CONFIGURE_DEPENDS re-globs on every build, so a fresh `cmake --build`
# picks up new files without you having to re-run `cmake -S . -B build`
# by hand (still required if you *remove* a file, per CMake docs).
# ------------------------------------------------------------------
file(GLOB_RECURSE CORE_SOURCES CONFIGURE_DEPENDS
  algorithm/*.cpp
  common/*.cpp
  math/*.cpp
  sim_interface/*.cpp
)

add_library(core STATIC ${CORE_SOURCES})
target_include_directories(core PUBLIC
  algorithm
  common
  math
  sim_interface
)
target_link_libraries(core PUBLIC mujoco_simulate mujoco::mujoco Threads::Threads)

# ------------------------------------------------------------------
# Tests
# ------------------------------------------------------------------
add_executable(init_mujoco_gui tests/init_mujoco_gui.cpp)
target_link_libraries(init_mujoco_gui PRIVATE core dl)
target_compile_definitions(init_mujoco_gui PRIVATE
  BIPED_MODEL_DIR="${CMAKE_SOURCE_DIR}/models/mjcf"
)

add_executable(init_mujoco_glfw_ui tests/init_mujoco_glfw_ui.cpp)
target_link_libraries(init_mujoco_glfw_ui PRIVATE core dl)
target_compile_definitions(init_mujoco_glfw_ui PRIVATE
  BIPED_MODEL_DIR="${CMAKE_SOURCE_DIR}/models/mjcf"
)
