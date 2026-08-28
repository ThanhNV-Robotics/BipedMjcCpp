cmake_minimum_required(VERSION 3.16)
project(BipedMjcCpp LANGUAGES CXX)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Vendored third-party dirs that are both a link target (contain a prebuilt
# .a/.so) and an include path (contain their own public headers).
set(dirEigen "third_party/eigen3")
set(dirGlfw "third_party/glfw")
set(dirPino "third_party/pinocchio")
set(dirQuill "third_party/quill")
set(dirQpOASES "third_party/qpOASES")
set(dirMatplotlib "third_party/matplotlibcpp")

# Include-only paths: extra header locations needed on top of the dirs above.
set(incFmt "${dirQuill}/quill/bundled")  # quill vendors its own fmt copy here
set(incMujoco "third_party/mujoco/include")
set(incMujocoSim "third_party/mujoco/simulate")  # SimUI wraps mujoco's own simulate.h/glfw_adapter.h

set(allDir ${dirEigen} ${dirGlfw} ${dirPino} ${dirQuill} ${dirQpOASES} ${dirMatplotlib})
set(allInc ${allDir} ${incFmt} ${incMujoco} ${incMujocoSim})

include_directories(${allInc})
include_directories("algorithm" "common" "math" "sim_interface")

# Pick which arch-suffixed variant of the vendored static libs to link
# (e.g. libpinocchio_lin_x64.a vs libpinocchio_lin_arm64.a).
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  set(archSuffix "lin_arm64")
else()
  set(archSuffix "lin_x64")
endif()

# MuJoCo and qpOASES each ship as a single shared library per arch that
# doesn't live in a system lib directory (qpOASES's, unlike the other
# vendored libs, sits under an arch-suffixed subdirectory rather than
# having the suffix baked into its filename), so bake both locations into
# the executables' runtime search path.
set(CMAKE_BUILD_RPATH
  "${CMAKE_SOURCE_DIR}/third_party/mujoco/lib"
  "${CMAKE_SOURCE_DIR}/${dirQpOASES}/${archSuffix}"
)
link_directories(${dirGlfw} ${dirPino} ${dirQuill} "third_party/mujoco/lib" "${dirQpOASES}/${archSuffix}")

# Pinocchio's URDF parser needs Boost::filesystem/system and urdfdom.
# third_party/boost only vendors 1.76 headers, which don't compile against
# this system's GCC 13/libstdc++, so Boost is resolved via find_package()
# against the system install (1.83) instead of being added to the include
# path; urdfdom is likewise resolved against the system package (via
# pkg-config) rather than the vendored third_party/urdfdom static libs.
# OpenGL is needed to build mujoco's own "simulate" GUI from source.
# Python3+NumPy are needed by matplotlibcpp, which embeds a Python
# interpreter to drive matplotlib.
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)
find_package(PkgConfig REQUIRED)
pkg_check_modules(urdfdom REQUIRED IMPORTED_TARGET urdfdom)
find_package(Threads REQUIRED)
find_package(OpenGL REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Development NumPy)

# yaml-cpp is the one dependency not vendored as a prebuilt lib under
# third_party/ -- it's fetched and built from source instead, so this is
# the one target in the project that needs network access on a fresh
# `cmake` configure.
include(FetchContent)
FetchContent_Declare(
  yaml-cpp
  GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
  GIT_TAG 0.8.0
)
FetchContent_MakeAvailable(yaml-cpp)

# MuJoCo's own "simulate" GUI has no prebuilt lib, so build it from the
# vendored source under third_party/mujoco/simulate. sim_interface/SimUI
# wraps this to visualize an externally-owned mjModel/mjData. The
# Screenshot-to-PNG feature (and its lodepng dependency) was patched out of
# our copy of simulate.cc -- not needed, and one less vendored library.
add_library(mujoco_simulate STATIC
  "${incMujocoSim}/simulate.cc"
  "${incMujocoSim}/glfw_adapter.cc"
  "${incMujocoSim}/glfw_dispatch.cc"
  "${incMujocoSim}/platform_ui_adapter.cc"
)
target_link_libraries(mujoco_simulate mujoco glfw3_${archSuffix} OpenGL::GL Threads::Threads dl)

# matplotlibcpp (header-only, common/matplotlibcpp.h) wraps the
# Python/matplotlib C API for plotting from C++. common/ is already on the
# include path, so this interface target only needs to supply the
# Python/NumPy link -- opt in per executable with
# target_link_libraries(<tgt> PRIVATE matplotlibcpp), since not every
# target that links core needs a Python dependency.
add_library(matplotlibcpp INTERFACE)
target_link_libraries(matplotlibcpp INTERFACE Python3::Python Python3::NumPy)

# Collect this project's own sources into a single core library so every
# demo/test executable can link against "core" instead of repeating the
# source list. GLOB_RECURSE + CONFIGURE_DEPENDS re-globs on every build, so
# a fresh `cmake --build` picks up new files without you having to re-run
# `cmake -S . -B build` by hand (still required if you *remove* a file,
# per CMake docs).
file(GLOB_RECURSE CORE_SOURCES CONFIGURE_DEPENDS
  "algorithm/*.cpp" "common/*.cpp" "math/*.cpp" "sim_interface/*.cpp"
)

# Vendored static libs core links against, before arch-suffixing.
set(corLibs pinocchio quill)

# Expand each name above into its arch-specific filename, e.g.
# "pinocchio" -> "pinocchio_lin_x64" (matches libpinocchio_lin_x64.a).
set(sysCoreLibs)
foreach(lib ${corLibs})
  list(APPEND sysCoreLibs ${lib}_${archSuffix})
endforeach()

# Build the core control library and link it against everything the
# project's own sources (algorithm/common/math/sim_interface) need.
add_library(core STATIC ${CORE_SOURCES})
target_link_libraries(core PUBLIC
  ${sysCoreLibs} qpOASES
  mujoco_simulate mujoco glfw3_${archSuffix}
  Eigen3::Eigen Boost::filesystem Boost::system PkgConfig::urdfdom
  Threads::Threads dl yaml-cpp::yaml-cpp
)
target_compile_definitions(core PUBLIC PINOCCHIO_STATIC)

# Demo/test executables. Each links against "core", which transitively
# carries all of core's own dependencies.
add_executable(float_control demo/float_control.cpp)
target_link_libraries(float_control PRIVATE core)

add_executable(walk_wbc demo/walk_wbc.cpp)
target_link_libraries(walk_wbc PRIVATE core)

add_executable(test_state_estimator tests/test_state_estimator.cpp)
target_link_libraries(test_state_estimator PRIVATE core)

add_executable(test_joystick_interpreter tests/test_joystick_interpreter.cpp)
target_link_libraries(test_joystick_interpreter PRIVATE core)

# add_executable(test_gait_scheduler tests/test_gait_scheduler.cpp)
# target_link_libraries(test_gait_scheduler PRIVATE core matplotlibcpp)

add_executable(test_foot_placement tests/test_foot_placement.cpp)
target_link_libraries(test_foot_placement PRIVATE core matplotlibcpp)

add_executable(test_robot_wrapper tests/test_robot_wrapper.cpp)
target_link_libraries(test_robot_wrapper PRIVATE core matplotlibcpp)

add_executable(test_pin_mcf_model tests/test_pin_mcf_model.cpp)
target_link_libraries(test_pin_mcf_model PRIVATE core matplotlibcpp)

add_executable(test_Assembling tests/test_Assembling.cpp)
target_link_libraries(test_Assembling PRIVATE core matplotlibcpp)

add_executable(test_WBCKin tests/test_WBCKin.cpp)
target_link_libraries(test_WBCKin PRIVATE core)

add_executable(test_cp_planning tests/test_cp_planning.cpp)
target_link_libraries(test_cp_planning PRIVATE core matplotlibcpp)
