set(target hyped_navigation)
file(GLOB headers "${CMAKE_CURRENT_SOURCE_DIR}/*.hpp")
file(GLOB code "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
add_library(${target} STATIC ${headers} ${code})
#find_package (Eigen3 3.4 REQUIRED NO_MODULE)
find_package (Eigen3 REQUIRED NO_MODULE)
include_directories(${target}
    INTERFACE
    "${CMAKE_SOURCE_DIR}/lib"
    "${CMAKE_CURRENT_SOURCE_DIR}"
)
target_link_libraries(${target} PUBLIC Eigen3::Eigen)
export(
    TARGETS ${target}
    NAMESPACE Hyped::
    FILE "${CMAKE_BINARY_DIR}/export/HypedNavigationConfig.cmake"
)
export(PACKAGE ${target})