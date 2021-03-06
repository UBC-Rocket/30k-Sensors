cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)
cmake_policy(SET CMP0076 NEW)
project(rocketos)


if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    set(CMAKE_CXX_FLAGS "-pthread -Wall -Wextra -pedantic -O0")
else()
    set(CMAKE_CXX_FLAGS "-pthread -static-libstdc++ -static-libgcc -Wall -Wextra -pedantic -O0")
endif()

# add_executable(demo main.cpp)

###
### Google Test
###

# Download and unpack googletest at configure time
configure_file(CMakeLists.txt.in googletest-download/CMakeLists.txt)
execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
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

# The gtest/gtest_main targets carry header search path
# dependencies automatically when using CMake 2.8.11 or
# later. Otherwise we have to add them here ourselves.
if (CMAKE_VERSION VERSION_LESS 2.8.11)
  include_directories("${gtest_SOURCE_DIR}/include")
endif()



###
### Link all tests here
###
enable_testing()

# e.g.
add_executable(testSchedule tests/test_schedule.cpp )
target_sources(testSchedule PUBLIC include/RocketSchedule/fake_clocks.cpp)
target_link_libraries(testSchedule gtest_main gmock_main)
target_include_directories(testSchedule PUBLIC include tests/support)
add_test(NAME TestSchedule COMMAND testSchedule)

# We can now call make test to run all the tests


# ###
# ### Code coverage?
# ###
# set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
# include(CodeCoverage)
# append_coverage_compiler_flags()
# setup_target_for_coverage_gcovr_html(
#     NAME ctest_coverage
#     EXECUTABLE
# )
