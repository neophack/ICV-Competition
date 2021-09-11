file(GLOB_RECURSE srcs *.h *.hh *i *.hpp *.c *.cc *.cpp)
file(GLOB_RECURSE headers *.h)
file(GLOB libs ${LIB_DIR}/*.*)

add_executable(PredefinedRoute ${srcs})

target_sources(PredefinedRoute PRIVATE "src/EGearPreRoute.cpp" ${headers})

source_group(TREE ${CMAKE_CURRENT_LIST_DIR} FILES ${srcs})

include_directories("${CMAKE_CURRENT_LIST_DIR}/include/")
include_directories("${INCLUDE_DIR}")

add_definitions(-DNOMINMAX)
add_definitions(-DUNICODE)
add_definitions(-D_UNICODE)

SetDefaultTargetProperties(PredefinedRoute)

IF (UNIX)
    target_link_libraries(PredefinedRoute debug ${LIB_DIR}/libSimOneIOAPI.so)
    target_link_libraries(PredefinedRoute optimized ${LIB_DIR}/libSimOneIOAPI.so -lpthread)
    target_link_libraries(PredefinedRoute debug /usr/lib/x86_64-linux-gnu/libcurl.so.4)
    target_link_libraries(PredefinedRoute optimized /usr/lib/x86_64-linux-gnu/libcurl.so.4)
    target_link_libraries(PredefinedRoute debug ${LIB_DIR}/libSSD.so)
    target_link_libraries(PredefinedRoute optimized ${LIB_DIR}/libSSD.so -lpthread)
    target_link_libraries(PredefinedRoute debug ${LIB_DIR}/libHDMapModule.so)
    target_link_libraries(PredefinedRoute optimized ${LIB_DIR}/libHDMapModule.so)

    add_custom_command(TARGET PredefinedRoute
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${LIB_DIR}/libSimOneIOAPI.so ${BINARY_OUT_PATH}/libSimOneIOAPI.so
        COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${LIB_DIR}/libSSD.so ${BINARY_OUT_PATH}/libSSD.so
        COMMAND ${CMAKE_COMMAND} -E copy_if_different  ${LIB_DIR}/libHDMapModule.so ${BINARY_OUT_PATH}/libHDMapModule.so
        )
    foreach(CurrentLibFile IN LISTS libs)
        add_custom_command(
            TARGET PredefinedRoute
            POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CurrentLibFile} ${BINARY_OUT_PATH}
            COMMENT "Copying lib: ${CurrentLibFile}"
            )
    endforeach()
ENDIF()
