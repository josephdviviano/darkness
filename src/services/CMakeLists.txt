# A monolith object library for all services

set(PLATFORM_SOURCES
    platform/PlatformService.cpp
    platform/PlatformService.h
    platform/Platform.cpp
    platform/Platform.h
)

# OS Specific code follows
if(UNIX)
    if(APPLE)
        # Apple uses the Unix platform base with macOS-specific paths
        list(APPEND PLATFORM_SOURCES platform/ApplePlatform.cpp platform/ApplePlatform.h)
    else()
        # Generic unix platform
        list(APPEND PLATFORM_SOURCES platform/UnixPlatform.cpp platform/UnixPlatform.h)
    endif()
else()
    if(WIN32)
        list(APPEND PLATFORM_SOURCES platform/Win32Platform.cpp platform/Win32Platform.h)
    else()
        message(FATAL_ERROR "Unknown platform encountered! Please consider helping us by providing your platform specific code!")
    endif()
endif()

# Core services that work without a renderer
set(HEADLESS_SERVICES_SOURCES
    config/ConfigService.cpp
    config/ConfigService.h
    database/DatabaseService.cpp
    database/DatabaseService.h
    game/GameService.cpp
    game/GameService.h
    inherit/ArchetypeInheritor.cpp
    inherit/ArchetypeInheritor.h
    inherit/CachedInheritor.cpp
    inherit/CachedInheritor.h
    inherit/InheritCommon.h
    inherit/InheritService.cpp
    inherit/InheritService.h
    inherit/NeverInheritor.cpp
    inherit/NeverInheritor.h
    link/LinkCommon.h
    link/LinkService.cpp
    link/LinkService.h
    link/Relation.cpp
    link/Relation.h
    loop/LoopService.cpp
    loop/LoopService.h
    object/ObjectService.cpp
    object/ObjectService.h
    object/PositionPropertyStorage.cpp
    object/PositionPropertyStorage.h
    object/SymNamePropertyStorage.cpp
    object/SymNamePropertyStorage.h
    physics/PhysBSPModel.cpp
    physics/PhysBSPModel.h
    physics/PhysCommon.cpp
    physics/PhysCommon.h
    physics/PhysControls.h
    physics/PhysicsService.cpp
    physics/PhysicsService.h
    physics/PhysModel.cpp
    physics/PhysModel.h
    physics/PhysModels.cpp
    physics/PhysModels.h
    physics/PhysOBBModel.cpp
    physics/PhysOBBModel.h
    physics/PhysSphereModel.cpp
    physics/PhysSphereModel.h
    ${PLATFORM_SOURCES}
    property/PropertyCommon.h
    property/Property.cpp
    property/Property.h
    property/PropertyService.cpp
    property/PropertyService.h
    room/RoomCommon.h
    room/Room.cpp
    room/Room.h
    room/RoomPortal.cpp
    room/RoomPortal.h
    room/RoomService.cpp
    room/RoomService.h
    ServiceCommon.h
    sim/SimService.cpp
    sim/SimService.h
)

# Rendering services that require Ogre3D (or eventually bgfx)
set(RENDER_SERVICES_SOURCES
    camera/CameraService.cpp
    camera/CameraService.h
    draw/DrawBuffer.cpp
    draw/DrawBuffer.h
    draw/DrawCommon.cpp
    draw/DrawCommon.h
    draw/DrawOperation.cpp
    draw/DrawOperation.h
    draw/DrawService.cpp
    draw/DrawService.h
    draw/DrawSheet.cpp
    draw/DrawSheet.h
    draw/FontDrawSource.cpp
    draw/FontDrawSource.h
    draw/RenderedImage.cpp
    draw/RenderedImage.h
    draw/RenderedLabel.cpp
    draw/RenderedLabel.h
    draw/RenderedRect.cpp
    draw/RenderedRect.h
    draw/TextureAtlas.cpp
    draw/TextureAtlas.h
    gui/ConsoleGUI.cpp
    gui/ConsoleGUI.h
    gui/GUIService.cpp
    gui/GUIService.h
    input/InputEventMapper.cpp
    input/InputEventMapper.h
    input/InputService.cpp
    input/InputService.h
    light/LightService.cpp
    light/LightService.h
    material/MaterialService.cpp
    material/MaterialService.h
    render/EntityMaterialInstance.cpp
    render/EntityMaterialInstance.h
    render/EntityInfo.cpp
    render/EntityInfo.h
    render/HasRefsProperty.cpp
    render/HasRefsProperty.h
    render/jorge.h
    render/MaterialInstance.cpp
    render/MaterialInstance.h
    render/ModelNameProperty.cpp
    render/ModelNameProperty.h
    render/ModelScaleProperty.cpp
    render/ModelScaleProperty.h
    render/RenderAlphaProperty.cpp
    render/RenderAlphaProperty.h
    render/RenderedProperty.cpp
    render/RenderedProperty.h
    render/RenderService.cpp
    render/RenderService.h
    render/RenderTypeProperty.cpp
    render/RenderTypeProperty.h
    render/SubEntityMaterialInstance.cpp
    render/SubEntityMaterialInstance.h
    render/ZBiasProperty.cpp
    render/ZBiasProperty.h
    worldrep/LightmapAtlas.cpp
    worldrep/LightmapAtlas.h
    worldrep/LightsForCell.cpp
    worldrep/LightsForCell.h
    worldrep/WorldRepService.cpp
    worldrep/WorldRepService.h
    worldrep/WRCell.cpp
    worldrep/WRCell.h
    worldrep/WRCommon.h
    worldrep/WRTypes.h
)

add_library(OpdeServices OBJECT ${HEADLESS_SERVICES_SOURCES})

target_link_libraries(OpdeServices PUBLIC OpdeBase)

target_include_directories(OpdeServices PUBLIC
    ${ODE_INCLUDE_DIR}
    ${OPDE_SOURCE_DIR}/src/services
)

target_compile_options(OpdeServices PRIVATE
    $<$<OR:$<CXX_COMPILER_ID:GNU>,$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:AppleClang>>:
        -Wall -fno-strict-aliasing -fvisibility=default>
)
