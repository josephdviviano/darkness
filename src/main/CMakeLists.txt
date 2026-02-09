# CLI tool for loading/inspecting Dark Engine databases
add_executable(darknessHeadless DarknessHeadless.cpp)

target_include_directories(darknessHeadless PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(darknessHeadless PRIVATE
    DarknessBase
    DarknessServices
    ${ODE_LIBRARIES}
    ${SDL2_LIBRARIES}
)

install(TARGETS darknessHeadless
    RUNTIME DESTINATION ${DARKNESS_BINARY_INSTALL_DIR}
)

# SDL2 + bgfx render window (task 6a)
find_package(bgfx CONFIG REQUIRED)
find_package(SDL2 CONFIG REQUIRED)
find_package(zziplib CONFIG REQUIRED)
find_package(yaml-cpp CONFIG REQUIRED)

add_executable(darknessRender DarknessRender.cpp)

target_include_directories(darknessRender PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(darknessRender PRIVATE
    DarknessBase
    bgfx::bgfx
    bgfx::bx
    bgfx::bimg
    SDL2::SDL2main
    SDL2::SDL2-static
    zziplib::libzzip
    yaml-cpp::yaml-cpp
)

install(TARGETS darknessRender
    RUNTIME DESTINATION ${DARKNESS_BINARY_INSTALL_DIR}
)
