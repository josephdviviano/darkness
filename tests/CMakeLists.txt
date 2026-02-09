# Unit tests using Catch2 v3
find_package(Catch2 3 CONFIG REQUIRED)
find_package(yaml-cpp CONFIG REQUIRED)

add_executable(darkness_tests
    test_render_config.cpp
)

target_link_libraries(darkness_tests PRIVATE
    Catch2::Catch2WithMain
    yaml-cpp::yaml-cpp
)

# RenderConfig.h is header-only, lives in src/main/
target_include_directories(darkness_tests PRIVATE
    ${CMAKE_SOURCE_DIR}/src/main
)

# Auto-discover Catch2 test cases for CTest
list(APPEND CMAKE_MODULE_PATH ${Catch2_DIR})
include(Catch)
catch_discover_tests(darkness_tests)
