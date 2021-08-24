aux_source_directory(./src SRCS)
add_executable(AEB ${SRCS})

include_directories("${INCLUDE_DIR}")

source_group(TREE ${CMAKE_CURRENT_LIST_DIR} FILES ${SRCS})

setdefaulttargetproperties(AEB)

add_definitions(-DNOMINMAX)
add_definitions(-DUNICODE)
add_definitions(-D_UNICODE)

target_link_libraries(AEB debug ${LIB_DIR}/libSimOneIOAPI.so)
target_link_libraries(AEB optimized ${LIB_DIR}/libSimOneIOAPI.so -lpthread)
target_link_libraries(AEB debug ${LIB_DIR}/libSSD.so)
target_link_libraries(AEB optimized ${LIB_DIR}/libSSD.so -lpthread)
target_link_libraries(AEB debug ${LIB_DIR}/libHDMapModule.so)
target_link_libraries(AEB optimized ${LIB_DIR}/libHDMapModule.so)

add_custom_command(
  TARGET AEB
  POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_if_different ${LIB_DIR}/libSimOneIOAPI.so
          ${BINARY_OUT_PATH}/libSimOneIOAPI.so
  COMMAND ${CMAKE_COMMAND} -E copy_if_different ${LIB_DIR}/libSSD.so
          ${BINARY_OUT_PATH}/libSSD.so
  COMMAND ${CMAKE_COMMAND} -E copy_if_different ${LIB_DIR}/libHDMapModule.so
          ${BINARY_OUT_PATH}/libHDMapModule.so
  #############################################################################
  COMMAND ${CMAKE_COMMAND} -E copy_if_different ${LIB_DIR}/libavformat.so.58
          ${BINARY_OUT_PATH}/libavformat.so.58
  COMMAND ${CMAKE_COMMAND} -E copy_if_different ${LIB_DIR}/libavcodec.so.58
          ${BINARY_OUT_PATH}/libavcodec.so.58
  COMMAND ${CMAKE_COMMAND} -E copy_if_different ${LIB_DIR}/libavutil.so.56
          ${BINARY_OUT_PATH}/libavutil.so.56
  COMMAND ${CMAKE_COMMAND} -E copy_if_different ${LIB_DIR}/libswresample.so.3
          ${BINARY_OUT_PATH}/libswresample.so.3
  COMMAND ${CMAKE_COMMAND} -E copy_if_different ${LIB_DIR}/libswscale.so.5
          ${BINARY_OUT_PATH}/libswscale.so.5
  COMMAND ${CMAKE_COMMAND} -E copy_if_different ${LIB_DIR}/libx264.so.148
          ${BINARY_OUT_PATH}/libx264.so.148
  COMMAND
    ${CMAKE_COMMAND} -E copy_if_different ${LIB_DIR}/libSimOneStreamingIOAPI.so
    ${BINARY_OUT_PATH}/libSimOneStreamingIOAPI.so)
