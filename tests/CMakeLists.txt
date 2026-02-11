# Unit tests using Catch2 v3
find_package(Catch2 3 CONFIG REQUIRED)
find_package(yaml-cpp CONFIG REQUIRED)
find_package(zziplib CONFIG REQUIRED)

add_executable(darkness_tests
    test_render_config.cpp
    test_typed_properties.cpp
    test_world_query.cpp
)

target_link_libraries(darkness_tests PRIVATE
    Catch2::Catch2WithMain
    DarknessBase
    DarknessServices
    yaml-cpp::yaml-cpp
    zziplib::libzzip
    ${ODE_LIBRARIES}
)

# Include paths for all headers used by property tests
target_include_directories(darkness_tests PRIVATE
    ${CMAKE_SOURCE_DIR}/src/main
    ${CMAKE_SOURCE_DIR}/src/services
    ${CMAKE_SOURCE_DIR}/src/base
    ${CMAKE_SOURCE_DIR}/src/base/dyntype
)

# Point tests to fixture zip and schema scripts
target_compile_definitions(darkness_tests PRIVATE
    FIXTURES_DIR="${CMAKE_SOURCE_DIR}/tests/fixtures"
    SCRIPTS_DIR="${CMAKE_SOURCE_DIR}/scripts/thief2"
)

# Auto-discover Catch2 test cases for CTest
list(APPEND CMAKE_MODULE_PATH ${Catch2_DIR})
include(Catch)
catch_discover_tests(darkness_tests)
