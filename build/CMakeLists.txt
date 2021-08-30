cmake_minimum_required(VERSION 3.8.0)

# compiler and flags
set(CMAKE_C_COMPILER "/usr/bin/clang")
set(CMAKE_CXX_COMPILER "/usr/bin/clang++")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++ -I/usr/include/libcxxabi -std=c++14")
set(CMAKE_LINKER_FLAGS "${CMAKE_LINKER_FLAGS} -stdlib=libc++ -lc++abi")

project(ICV-Competition VERSION 0.1.0)

# include and lib
set(LIB_DIR "${CMAKE_CURRENT_LIST_DIR}/../lib")
set(INCLUDE_DIR "${CMAKE_CURRENT_LIST_DIR}/../include")

# varibles
set(WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/..)
set(BINARY_OUT_PATH ${WORKING_DIRECTORY}/bin)


set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set_property(GLOBAL PROPERTY PREDEFINED_TARGETS_FOLDER "CMakeTargets")

# pthread
set(THREADS_PREFER_PTHREAD_FLAG ON)


function(SetDefaultTargetProperties target)
  set_target_properties(${target} PROPERTIES CXX_STANDARD 14)
  set_target_properties(${target} PROPERTIES RUNTIME_OUTPUT_DIRECTORY
                                             ${BINARY_OUT_PATH})
endfunction()

# subprojects
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../ADAS/AEB AEB)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../ADAS/AVP AVP)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../ABC ABC)
