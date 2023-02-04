set(target hyped_core)
file(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
file(GLOB code "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
add_library(${target} STATIC ${headers} ${code})
target_include_directories(${target}
    INTERFACE "${CMAKE_SOURCE_DIR}/lib"
)
export(
    TARGETS ${target}
    NAMESPACE Hyped::
    FILE "${CMAKE_BINARY_DIR}/export/HypedCoreConfig.cmake"
)
export(PACKAGE ${target})