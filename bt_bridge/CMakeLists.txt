cmake_minimum_required(VERSION 3.15)
project(bt_bridge LANGUAGES CXX)

# See docs/behavior-tree-migration.md for the full design/rationale. This extension is part of
# the live repo (bind-mounted, per docker-compose.yaml -- never baked into the image), so unlike
# BehaviorTree.CPP/pybind11/ZeroMQ/SQLite themselves (installed into the isaac-cobot-curobo image
# at build time, see docker/Dockerfile.curobo), it is built once per container by running
# scripts/build_bt_bridge.sh, not during `docker build`.

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Must resolve to Isaac Sim's own bundled interpreter (/isaac-sim/kit/python/, Python 3.11 as of
# Isaac Sim 5.1.0), not system Python -- the same wrong-interpreter trap Dockerfile.curobo already
# routes around for torch. scripts/build_bt_bridge.sh passes this explicitly; the default here is a
# convenience for a manual `cmake -S bt_bridge -B bt_bridge/build` invocation.
if(NOT DEFINED Python3_EXECUTABLE)
  set(Python3_EXECUTABLE "$ENV{ISAACSIM_ROOT_PATH}/python.sh" CACHE FILEPATH "Interpreter to build/link against")
endif()

# COMPONENTS must include Interpreter, not just Development -- otherwise FindPython3 ignores the
# Python3_EXECUTABLE hint above and resolves Development against whatever system Python it finds
# first (confirmed live: without Interpreter here, this silently picked up system Python 3.12's
# headers instead of Isaac Sim's bundled 3.11, producing an extension the real interpreter can't
# import).
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)
find_package(behaviortree_cpp REQUIRED)

# Module name is mefron_bt_bridge, not bt_bridge -- see bt_bridge.cpp's own PYBIND11_MODULE
# comment for why (this directory's own name collides with it as an implicit namespace package).
pybind11_add_module(mefron_bt_bridge src/bt_bridge.cpp)
target_link_libraries(mefron_bt_bridge PRIVATE BT::behaviortree_cpp)
