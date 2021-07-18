cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)
cmake_policy(SET CMP0076 NEW)

project(FLARE)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set(CMAKE_CXX_FLAGS "-pthread -Wall -Wextra -g")
else()
    set(CMAKE_CXX_FLAGS "-pthread -static-libstdc++ -static-libgcc -Wall -Wextra -g")
endif()

add_executable(TantalusStage1 common/src/avionics.cpp)
add_executable(TantalusStage2 common/src/avionics.cpp)
add_executable(Hollyburn common/src/avionics.cpp)

add_subdirectory(common)
add_subdirectory(envs/x86)
add_subdirectory(rockets/tantalus_stage_1)
add_subdirectory(rockets/tantalus_stage_2)
add_subdirectory(rockets/hollyburn)

include_directories(
    common/include
    common/lib/RocketScheduler/include
    common/lib
    envs/x86/include
    envs/x86/lib
    extra
)

# Rocket-specific directories
target_include_directories(TantalusStage1 PUBLIC
  rockets/tantalus_stage_1/include
)
target_sources(TantalusStage1 PUBLIC extra/src/altitude_avg.cpp)

target_include_directories(TantalusStage2 PUBLIC
rockets/tantalus_stage_2/include
)
target_sources(TantalusStage2 PUBLIC extra/src/altitude_avg.cpp)


target_include_directories(Hollyburn PUBLIC
rockets/hollyburn/include
)

target_sources(Hollyburn PUBLIC extra/src/altitude_avg.cpp)

###
### Setup Radio defines
###

add_compile_definitions(RADIO_CONFIG_PACKET_SIM_ACTIVE=1)
target_compile_definitions(TantalusStage1 PUBLIC
    RADIO_CONFIG_PACKET_ROCKET_ID=0
)
target_compile_definitions(TantalusStage2 PUBLIC
    RADIO_CONFIG_PACKET_ROCKET_ID=1
)

target_compile_definitions(Hollyburn PUBLIC
    RADIO_CONFIG_PACKET_ROCKET_ID=3
)

###
### Setup Git version
###
message(STATUS "Resolving GIT Version")
message("Directory: ${CMAKE_CURRENT_LIST_DIR}")
set(_git_build_version "unknown")

find_package(Python3 COMPONENTS Interpreter)
if(Python3_FOUND)
execute_process(
  COMMAND ${Python3_EXECUTABLE} "${CMAKE_CURRENT_LIST_DIR}/../custom_tools/get_submodule.py"
  WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
)
execute_process(
  COMMAND ${Python3_EXECUTABLE} "${CMAKE_CURRENT_LIST_DIR}/../custom_tools/git_rev_macro.py" "x86"
  OUTPUT_VARIABLE _git_build_version
  WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
  )
  message(STATUS "Git hash: ${_git_build_version}")
  add_compile_definitions(RADIO_CONFIG_PACKET_VERSION_STR="${_git_build_version}")
else()
  message(SEND_ERROR "Python not found; unable to set Radio Config data")
endif()

###
### Google Test
###

# Download and unpack googletest at configure time
# -Wno-dev suppresses warnings - CMake gets confused when you do this, I guess
configure_file(CMakeLists.txt.in googletest-download/CMakeLists.txt)
execute_process(COMMAND ${CMAKE_COMMAND} -Wno-dev -G "${CMAKE_GENERATOR}" .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/googletest-download )
if(result)
  message(FATAL_ERROR "CMake step for googletest failed: ${result}")
endif()

execute_process(COMMAND ${CMAKE_COMMAND} --build .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/googletest-download )
if(result)
  message(FATAL_ERROR "Build step for googletest failed: ${result}")
endif()

# Prevent overriding the parent project's compiler/linker
# settings on Windows
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

# Add googletest directly to our build. This defines
# the gtest and gtest_main targets.
add_subdirectory(${CMAKE_CURRENT_BINARY_DIR}/googletest-src
                 ${CMAKE_CURRENT_BINARY_DIR}/googletest-build
                 EXCLUDE_FROM_ALL)
# Note 2020-12-20 - above add_subdirectory call throws CMake versioning errors.
# There's a recent (as of writing) pull request in googletest upstream
# https://github.com/google/googletest/pull/3094

# The gtest/gtest_main targets carry header search path
# dependencies automatically when using CMake 2.8.11 or
# later. Otherwise we have to add them here ourselves.
if (CMAKE_VERSION VERSION_LESS 2.8.11)
  include_directories("${gtest_SOURCE_DIR}/include")
endif()

enable_testing()

###
### Link all tests here
###
### Note: Compile project in build folder to keep unnecessarily files out from main directory
###

# Helper function that does some boilderplate
function(ROCKET_CREATE_TEST TEST_NAME)
    add_executable(testexe_${TEST_NAME}
        tests/unit_${TEST_NAME}
    )
    target_link_libraries(testexe_${TEST_NAME} gtest_main)
    target_include_directories(testexe_${TEST_NAME} BEFORE PUBLIC tests/fake_include)
    add_test(NAME test_${TEST_NAME} COMMAND testexe_${TEST_NAME})
endfunction()

ROCKET_CREATE_TEST(LedBlinker)
target_sources(testexe_LedBlinker PUBLIC common/src/tasks/led_blinker.cpp)

ROCKET_CREATE_TEST(RepeatChecksState)

ROCKET_CREATE_TEST(States)

ROCKET_CREATE_TEST(RestartCamera)
target_sources(testexe_RestartCamera PUBLIC common/src/tasks/restart_camera.cpp)
target_sources(testexe_RestartCamera PUBLIC common/lib/RocketScheduler/include/RocketSchedule/fake_clocks.cpp)

ROCKET_CREATE_TEST(Barometric)
target_sources(testexe_Barometric PUBLIC common/lib/CalcHelpers/barometric.cpp)

ROCKET_CREATE_TEST(CalcHelpers)

ROCKET_CREATE_TEST(Estimators)
target_sources(testexe_Estimators PUBLIC
    common/lib/CalcHelpers/barometric.cpp
    )

ROCKET_CREATE_TEST(Roar)
target_Sources(testexe_Roar PUBLIC
    common/src/roar/buffer.cpp
)

ROCKET_CREATE_TEST(Log)
target_sources(testexe_Log PUBLIC
    common/src/log.cpp
    )

ROCKET_CREATE_TEST(Radio)
target_sources(testexe_Radio PUBLIC
    common/src/radio.cpp
    common/src/roar/buffer.cpp
    common/src/XBee.cpp
    common/src/sensor_collection.cpp
    common/src/ignitor_collection.cpp
    common/src/status.cpp
    common/src/hardware/ignitor.cpp
    common/src/log.cpp

    envs/x86/lib/HAL/port_impl.cpp
    envs/x86/lib/HAL/pin_util.cpp
)


# We can now call make test to run all the tests
