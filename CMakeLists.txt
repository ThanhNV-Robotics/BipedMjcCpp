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
# Pinocchio (prebuilt static library vendored under third_party/pinocchio).
# Pinocchio itself needs Eigen, Boost (filesystem/system), and urdfdom
# (for URDF parsing) -- all available as system packages, so we just
# find them and attach them to the imported target's INTERFACE.
# ------------------------------------------------------------------
set(PINOCCHIO_DIR ${CMAKE_SOURCE_DIR}/third_party/pinocchio)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)
find_package(PkgConfig REQUIRED)
pkg_check_modules(urdfdom REQUIRED IMPORTED_TARGET urdfdom)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  set(PINOCCHIO_LIB ${PINOCCHIO_DIR}/libpinocchio_lin_arm64.a)
else()
  set(PINOCCHIO_LIB ${PINOCCHIO_DIR}/libpinocchio_lin_x64.a)
endif()

add_library(pinocchio::pinocchio STATIC IMPORTED)
set_target_properties(pinocchio::pinocchio PROPERTIES
  IMPORTED_LOCATION ${PINOCCHIO_LIB}
  INTERFACE_INCLUDE_DIRECTORIES ${PINOCCHIO_DIR}
)
target_link_libraries(pinocchio::pinocchio INTERFACE
  Eigen3::Eigen
  Boost::filesystem
  Boost::system
  PkgConfig::urdfdom
)
target_compile_definitions(pinocchio::pinocchio INTERFACE PINOCCHIO_STATIC)

# ------------------------------------------------------------------
# jsoncpp (prebuilt static library vendored under third_party/jsoncpp)
# ------------------------------------------------------------------
set(JSONCPP_DIR ${CMAKE_SOURCE_DIR}/third_party/jsoncpp)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  set(JSONCPP_LIB ${JSONCPP_DIR}/libjsoncpp_lin_arm64.a)
else()
  set(JSONCPP_LIB ${JSONCPP_DIR}/libjsoncpp_lin_x64.a)
endif()

add_library(jsoncpp::jsoncpp STATIC IMPORTED)
set_target_properties(jsoncpp::jsoncpp PROPERTIES
  IMPORTED_LOCATION ${JSONCPP_LIB}
  INTERFACE_INCLUDE_DIRECTORIES ${JSONCPP_DIR}
)

# ------------------------------------------------------------------
# Quill (prebuilt static library vendored under third_party/quill).
# Quill bundles its own copy of fmt under quill/bundled/fmt, which is
# why "fmt/format.h" needs quill/bundled on the include path too.
# ------------------------------------------------------------------
set(QUILL_DIR ${CMAKE_SOURCE_DIR}/third_party/quill)
find_package(Threads REQUIRED)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  set(QUILL_LIB ${QUILL_DIR}/libquill_lin_arm64.a)
else()
  set(QUILL_LIB ${QUILL_DIR}/libquill_lin_x64.a)
endif()

add_library(quill::quill STATIC IMPORTED)
set_target_properties(quill::quill PROPERTIES
  IMPORTED_LOCATION ${QUILL_LIB}
  INTERFACE_INCLUDE_DIRECTORIES "${QUILL_DIR};${QUILL_DIR}/quill/bundled"
)
target_link_libraries(quill::quill INTERFACE Threads::Threads)

# ------------------------------------------------------------------
# GLFW (prebuilt static library vendored under third_party/glfw).
# The binary dlopen()s X11/Wayland at runtime rather than linking them
# directly (see its own undefined symbols), so dl + Threads are all it
# needs at link time -- both already available on any Linux desktop.
# ------------------------------------------------------------------
set(GLFW_DIR ${CMAKE_SOURCE_DIR}/third_party/glfw)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  set(GLFW_LIB ${GLFW_DIR}/libglfw3_lin_arm64.a)
else()
  set(GLFW_LIB ${GLFW_DIR}/libglfw3_lin_x64.a)
endif()

add_library(glfw::glfw STATIC IMPORTED)
set_target_properties(glfw::glfw PROPERTIES
  IMPORTED_LOCATION ${GLFW_LIB}
  INTERFACE_INCLUDE_DIRECTORIES ${GLFW_DIR}
)
target_link_libraries(glfw::glfw INTERFACE dl Threads::Threads)

# ------------------------------------------------------------------
# qpOASES (prebuilt shared library vendored under third_party/qpOASES,
# used by algorithm/mpc.cpp as the QP solver).
# ------------------------------------------------------------------
set(QPOASES_DIR ${CMAKE_SOURCE_DIR}/third_party/qpOASES)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  set(QPOASES_LIB ${QPOASES_DIR}/lin_arm64/libqpOASES.so)
else()
  set(QPOASES_LIB ${QPOASES_DIR}/lin_x64/libqpOASES.so)
endif()

add_library(qpOASES::qpOASES SHARED IMPORTED)
set_target_properties(qpOASES::qpOASES PROPERTIES
  IMPORTED_LOCATION ${QPOASES_LIB}
  INTERFACE_INCLUDE_DIRECTORIES ${QPOASES_DIR}
)

# ------------------------------------------------------------------
# Dependencies needed to build MuJoCo's own "simulate" GUI from source
# ------------------------------------------------------------------
find_package(OpenGL REQUIRED)

# MuJoCo's simulate UI sources, vendored under third_party/mujoco/simulate.
# The Screenshot-to-PNG feature (and its lodepng dependency) was patched out
# of our copy of simulate.cc -- not needed, and one less vendored library.
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
  glfw::glfw
  OpenGL::GL
  Threads::Threads
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
target_link_libraries(core PUBLIC mujoco_simulate mujoco::mujoco Threads::Threads Eigen3::Eigen jsoncpp::jsoncpp quill::quill pinocchio::pinocchio qpOASES::qpOASES)

# ------------------------------------------------------------------
# Tests
# ------------------------------------------------------------------
add_executable(init_mujoco_gui tests/init_mujoco_gui.cpp)
target_link_libraries(init_mujoco_gui PRIVATE core dl)

add_executable(init_mujoco_glfw_ui tests/init_mujoco_glfw_ui.cpp)
target_link_libraries(init_mujoco_glfw_ui PRIVATE core dl)

add_executable(init_pinocchio tests/init_pinocchio.cpp)
target_link_libraries(init_pinocchio PRIVATE pinocchio::pinocchio)

add_executable(joint_pd_control tests/joint_control/joint_pd_control.cpp)
target_link_libraries(joint_pd_control PRIVATE core dl)
target_link_libraries(joint_pd_control PRIVATE pinocchio::pinocchio)

add_executable(float_control demo/float_control.cpp)
target_link_libraries(float_control PRIVATE core dl)
target_link_libraries(float_control PRIVATE pinocchio::pinocchio)

add_executable(walk_in_air demo/walk_in_air.cpp)
target_link_libraries(walk_in_air PRIVATE core dl)
target_link_libraries(walk_in_air PRIVATE pinocchio::pinocchio)

add_executable(walk_wbc demo/walk_wbc.cpp)
target_link_libraries(walk_wbc PRIVATE core dl)
target_link_libraries(walk_wbc PRIVATE pinocchio::pinocchio)
