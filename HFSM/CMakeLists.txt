add_executable(HFSM "")

file(GLOB_RECURSE srcs *.hh *i *.hpp *.c *.cc *.cpp)
file(GLOB_RECURSE headers *.h)

target_sources(HFSM PRIVATE ${srcs})

source_group(TREE ${CMAKE_CURRENT_LIST_DIR} FILES ${srcs})

include_directories("${INCLUDE_DIR}")
include_directories(./src/kdtree)
include_directories(./src/header)
include_directories(./src/interface)


add_definitions(-DNOMINMAX)
add_definitions(-DUNICODE)
add_definitions(-D_UNICODE)

IF (MSVC)
    string(REPLACE "/MDd" "/MD" CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}")
ENDIF()

SetDefaultTargetProperties(HFSM)


target_link_libraries(HFSM debug ${LIB_DIR}/libSimOneIOAPI.so)
target_link_libraries(HFSM optimized ${LIB_DIR}/libSimOneIOAPI.so -lpthread)
target_link_libraries(HFSM debug ${LIB_DIR}/libSSD.so)
target_link_libraries(HFSM optimized ${LIB_DIR}/libSSD.so -lpthread)
target_link_libraries(HFSM debug ${LIB_DIR}/libHDMapModule.so)
target_link_libraries(HFSM optimized ${LIB_DIR}/libHDMapModule.so)

target_link_libraries(HFSM debug /usr/lib/x86_64-linux-gnu/libcurl.so.4)
target_link_libraries(HFSM optimized /usr/lib/x86_64-linux-gnu/libcurl.so.4)

add_custom_command(TARGET HFSM
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${LIB_DIR}/libSimOneIOAPI.so ${BINARY_OUT_PATH}/libSimOneIOAPI.so
    COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${LIB_DIR}/libSSD.so ${BINARY_OUT_PATH}/libSSD.so
    COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${LIB_DIR}/libHDMapModule.so ${BINARY_OUT_PATH}/libHDMapModule.so

    COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${LIB_DIR}/libavformat.so.58 ${BINARY_OUT_PATH}/libavformat.so.58
    COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${LIB_DIR}/libavcodec.so.58 ${BINARY_OUT_PATH}/libavcodec.so.58
    COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${LIB_DIR}/libavutil.so.56 ${BINARY_OUT_PATH}/libavutil.so.56
    COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${LIB_DIR}/libswresample.so.3 ${BINARY_OUT_PATH}/libswresample.so.3
    COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${LIB_DIR}/libswscale.so.5 ${BINARY_OUT_PATH}/libswscale.so.5
    COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${LIB_DIR}/libx264.so.148 ${BINARY_OUT_PATH}/libx264.so.148
    COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${LIB_DIR}/libSimOneStreamingIOAPI.so ${BINARY_OUT_PATH}/libSimOneStreamingIOAPI.so
)

