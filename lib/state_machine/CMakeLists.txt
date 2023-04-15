set(target hyped_statemachine)
file(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
file(GLOB code "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
find_package(Boost REQUIRED)
add_library(${target} STATIC ${headers} ${code})
include_directories(${target}
    INTERFACE
    "${CMAKE_SOURCE_DIR}/lib"
    "${CMAKE_CURRENT_SOURCE_DIR}"
)
target_link_libraries(
    ${target}
    Boost::boost
)
export(
    TARGETS ${target}
    NAMESPACE Hyped::
    FILE "${CMAKE_BINARY_DIR}/export/HypedStateMachineConfig.cmake"
)
export(PACKAGE ${target})