# N.B. This is a temporary flag. It only really applies to Linux, as Mac
# does not need X11.
option(RUN_X11_TESTS "Run tests that require X11" OFF)

include(CTest)

if(APPLE)
    set(FIND_PYTHON_EXECUTABLE_PATHS /usr/local/bin)
    set(FIND_PYTHON_INTERP_VERSION 3.9)
else()
    set(FIND_PYTHON_EXECUTABLE_PATHS /usr/bin)
    set(FIND_PYTHON_INTERP_VERSION 3.6)
endif()
find_program(PYTHON_EXECUTABLE NAMES python3
        PATHS "${FIND_PYTHON_EXECUTABLE_PATHS}"
        NO_DEFAULT_PATH
        )
find_package(PythonInterp ${FIND_PYTHON_INTERP_VERSION} MODULE REQUIRED)

execute_process(COMMAND ${PYTHON_EXECUTABLE}-config --exec-prefix
        OUTPUT_VARIABLE PYTHON_EXEC_PREFIX
        OUTPUT_STRIP_TRAILING_WHITESPACE
        )
list(APPEND CMAKE_PREFIX_PATH "${PYTHON_EXEC_PREFIX}")
find_package(PythonLibs ${FIND_PYTHON_INTERP_VERSION} MODULE REQUIRED)

find_package(drake CONFIG REQUIRED PATHS "~/env/drake")

get_filename_component(PYTHONPATH
        "${drake_DIR}/../../python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}/site-packages"
        REALPATH
        )

set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(GFLAGS_USE_TARGET_NAMESPACE OFF)
find_package(gflags CONFIG REQUIRED)

add_library(iiwa_common iiwa_common.cc iiwa_common.h)
target_link_libraries(iiwa_common drake::drake)

add_library(iiwa_lcm iiwa_lcm.cc iiwa_lcm.h)
target_link_libraries(iiwa_lcm drake::drake)

add_library(lcm_plan_interpolator lcm_plan_interpolator.cc lcm_plan_interpolator.h robot_plan_interpolator.cc robot_plan_interpolator.h )
target_link_libraries(lcm_plan_interpolator iiwa_lcm iiwa_common)

add_library(kuka_torque_controller kuka_torque_controller.cc kuka_torque_controller.h)
target_link_libraries(kuka_torque_controller drake::drake)

add_executable(iiwa_controller iiwa_controller.cc)
target_link_libraries(iiwa_controller iiwa_common lcm_plan_interpolator gflags)

add_executable(kuka_simulation kuka_simulation.cc)
target_link_libraries(kuka_simulation iiwa_common iiwa_lcm kuka_torque_controller gflags)

add_executable(kuka_plan_runner kuka_plan_runner.cc)
target_link_libraries(kuka_plan_runner drake::drake drake::drake-lcmtypes-cpp)

add_executable(move_iiwa_ee move_iiwa_ee.cc)
target_link_libraries(move_iiwa_ee drake::drake gflags)