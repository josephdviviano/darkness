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

# SDL2 + bgfx render window
find_package(bgfx CONFIG REQUIRED)
find_package(SDL2 CONFIG REQUIRED)
find_package(zziplib CONFIG REQUIRED)
find_package(yaml-cpp CONFIG REQUIRED)

# ── Cross-platform shader compilation ──
# Compiles .sc shaders to all backends supported on the build platform
# (Metal on macOS, GLSL+ESSL+SPIRV everywhere, DXBC on Windows).
# Output is embedded as C headers via bgfx's AS_HEADERS + BIN2C pipeline.

set(SHADER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../shaders)
set(GENERATED_SHADER_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated_shaders)

# Basic shader (flat colour) — uses varying.def.sc
bgfx_compile_shaders(
    TYPE VERTEX
    SHADERS ${SHADER_DIR}/vs_basic.sc
    VARYING_DEF ${SHADER_DIR}/varying.def.sc
    OUTPUT_DIR ${GENERATED_SHADER_DIR}
    OUT_FILES_VAR BASIC_VS_OUTPUTS
    INCLUDE_DIRS ${SHADER_DIR}
    AS_HEADERS
)
bgfx_compile_shaders(
    TYPE FRAGMENT
    SHADERS ${SHADER_DIR}/fs_basic.sc
    VARYING_DEF ${SHADER_DIR}/varying.def.sc
    OUTPUT_DIR ${GENERATED_SHADER_DIR}
    OUT_FILES_VAR BASIC_FS_OUTPUTS
    INCLUDE_DIRS ${SHADER_DIR}
    AS_HEADERS
)

# Textured shader — uses varying_textured.def.sc
bgfx_compile_shaders(
    TYPE VERTEX
    SHADERS ${SHADER_DIR}/vs_textured.sc
    VARYING_DEF ${SHADER_DIR}/varying_textured.def.sc
    OUTPUT_DIR ${GENERATED_SHADER_DIR}
    OUT_FILES_VAR TEXTURED_VS_OUTPUTS
    INCLUDE_DIRS ${SHADER_DIR}
    AS_HEADERS
)
bgfx_compile_shaders(
    TYPE FRAGMENT
    SHADERS ${SHADER_DIR}/fs_textured.sc
    VARYING_DEF ${SHADER_DIR}/varying_textured.def.sc
    OUTPUT_DIR ${GENERATED_SHADER_DIR}
    OUT_FILES_VAR TEXTURED_FS_OUTPUTS
    INCLUDE_DIRS ${SHADER_DIR}
    AS_HEADERS
)

# Lightmapped shader — uses varying_lightmapped.def.sc
bgfx_compile_shaders(
    TYPE VERTEX
    SHADERS ${SHADER_DIR}/vs_lightmapped.sc
    VARYING_DEF ${SHADER_DIR}/varying_lightmapped.def.sc
    OUTPUT_DIR ${GENERATED_SHADER_DIR}
    OUT_FILES_VAR LIGHTMAPPED_VS_OUTPUTS
    INCLUDE_DIRS ${SHADER_DIR}
    AS_HEADERS
)
bgfx_compile_shaders(
    TYPE FRAGMENT
    SHADERS ${SHADER_DIR}/fs_lightmapped.sc
    VARYING_DEF ${SHADER_DIR}/varying_lightmapped.def.sc
    OUTPUT_DIR ${GENERATED_SHADER_DIR}
    OUT_FILES_VAR LIGHTMAPPED_FS_OUTPUTS
    INCLUDE_DIRS ${SHADER_DIR}
    AS_HEADERS
)

# Lightmapped bicubic shader — cubic B-spline lightmap filtering
bgfx_compile_shaders(
    TYPE VERTEX
    SHADERS ${SHADER_DIR}/vs_lightmapped_bicubic.sc
    VARYING_DEF ${SHADER_DIR}/varying_lightmapped.def.sc
    OUTPUT_DIR ${GENERATED_SHADER_DIR}
    OUT_FILES_VAR LIGHTMAPPED_BICUBIC_VS_OUTPUTS
    INCLUDE_DIRS ${SHADER_DIR}
    AS_HEADERS
)
bgfx_compile_shaders(
    TYPE FRAGMENT
    SHADERS ${SHADER_DIR}/fs_lightmapped_bicubic.sc
    VARYING_DEF ${SHADER_DIR}/varying_lightmapped.def.sc
    OUTPUT_DIR ${GENERATED_SHADER_DIR}
    OUT_FILES_VAR LIGHTMAPPED_BICUBIC_FS_OUTPUTS
    INCLUDE_DIRS ${SHADER_DIR}
    AS_HEADERS
)

# Water shader — uses varying_water.def.sc
bgfx_compile_shaders(
    TYPE VERTEX
    SHADERS ${SHADER_DIR}/vs_water.sc
    VARYING_DEF ${SHADER_DIR}/varying_water.def.sc
    OUTPUT_DIR ${GENERATED_SHADER_DIR}
    OUT_FILES_VAR WATER_VS_OUTPUTS
    INCLUDE_DIRS ${SHADER_DIR}
    AS_HEADERS
)
bgfx_compile_shaders(
    TYPE FRAGMENT
    SHADERS ${SHADER_DIR}/fs_water.sc
    VARYING_DEF ${SHADER_DIR}/varying_water.def.sc
    OUTPUT_DIR ${GENERATED_SHADER_DIR}
    OUT_FILES_VAR WATER_FS_OUTPUTS
    INCLUDE_DIRS ${SHADER_DIR}
    AS_HEADERS
)

# Custom target so shaders rebuild when .sc sources change
add_custom_target(compile_shaders DEPENDS
    ${BASIC_VS_OUTPUTS} ${BASIC_FS_OUTPUTS}
    ${TEXTURED_VS_OUTPUTS} ${TEXTURED_FS_OUTPUTS}
    ${LIGHTMAPPED_VS_OUTPUTS} ${LIGHTMAPPED_FS_OUTPUTS}
    ${LIGHTMAPPED_BICUBIC_VS_OUTPUTS} ${LIGHTMAPPED_BICUBIC_FS_OUTPUTS}
    ${WATER_VS_OUTPUTS} ${WATER_FS_OUTPUTS}
)

add_executable(darknessRender DarknessRender.cpp)
add_dependencies(darknessRender compile_shaders)

target_include_directories(darknessRender PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${GENERATED_SHADER_DIR}
)

target_link_libraries(darknessRender PRIVATE
    DarknessBase
    DarknessServices
    bgfx::bgfx
    bgfx::bx
    bgfx::bimg
    SDL2::SDL2main
    SDL2::SDL2-static
    zziplib::libzzip
    yaml-cpp::yaml-cpp
    ${ODE_LIBRARIES}
)

install(TARGETS darknessRender
    RUNTIME DESTINATION ${DARKNESS_BINARY_INSTALL_DIR}
)
