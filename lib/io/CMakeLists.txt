set(target hyped_io)
file(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
file(GLOB code "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
add_library(${target} STATIC ${headers} ${code})
include_directories(${target}
    INTERFACE
    "${CMAKE_SOURCE_DIR}/lib"
    "${CMAKE_CURRENT_SOURCE_DIR}"
)
export(
    TARGETS ${target}
    NAMESPACE Hyped::
    FILE "${CMAKE_BINARY_DIR}/export/HypedIOConfig.cmake"
)
export(PACKAGE ${target})