# CLI tool for loading/inspecting Dark Engine databases
add_executable(opdeHeadless OpdeHeadless.cpp)

target_include_directories(opdeHeadless PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(opdeHeadless PRIVATE
    OpdeBase
    OpdeServices
    ${ODE_LIBRARIES}
    ${SDL2_LIBRARIES}
)

install(TARGETS opdeHeadless
    RUNTIME DESTINATION ${OPDE_BINARY_INSTALL_DIR}
)

# SDL2 + bgfx render window (task 6a)
find_package(bgfx CONFIG REQUIRED)
find_package(SDL2 CONFIG REQUIRED)
find_package(zziplib CONFIG REQUIRED)
find_package(yaml-cpp CONFIG REQUIRED)

add_executable(opdeRender OpdeRender.cpp)

target_include_directories(opdeRender PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(opdeRender PRIVATE
    OpdeBase
    bgfx::bgfx
    bgfx::bx
    bgfx::bimg
    SDL2::SDL2main
    SDL2::SDL2-static
    zziplib::libzzip
    yaml-cpp::yaml-cpp
)

install(TARGETS opdeRender
    RUNTIME DESTINATION ${OPDE_BINARY_INSTALL_DIR}
)
