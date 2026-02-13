set(DARKNESS_BASE_SOURCES
    BitArray.h
    Callback.h
    DarknessMath.h
    Matrix3.h
    Matrix4.h
    MessageSource.h
    NonCopyable.h
    DarknessException.cpp
    DarknessException.h
    DarknessSingleton.h
    LGPalette.h
    LGPalette.cpp
    Plane.h
    PrioritizedMessageSource.h
    Quaternion.h
    SharedPtr.h
    StringTokenizer.h
    tracer.cpp
    tracer.h
    ValueChangeRequest.h
    Vector2.h
    Vector3.h
    console/ConsoleBackend.cpp
    console/ConsoleBackend.h
    console/ConsoleCommandListener.cpp
    console/ConsoleCommandListener.h
    DarkCommon.h
    dyntype/DataStorage.cpp
    dyntype/DataStorage.h
    dyntype/DTHelpers.h
    dyntype/Variant.cpp
    dyntype/Variant.h
    dyntype/Serializer.cpp
    dyntype/Serializer.h
    dyntype/SingleFieldDataStorage.h
    dyntype/StructDataStorage.h
    file/darkdb.cpp
    file/darkdb.h
    file/FileCompat.cpp
    file/FileCompat.h
    file/File.cpp
    file/FileGroup.cpp
    file/FileGroup.h
    file/File.h
    FreeSpaceInfo.h
    Iterator.h
    logger/filelog.cpp
    logger/filelog.h
    logger/logger.cpp
    logger/logger.h
    logger/stdlog.cpp
    logger/stdlog.h
    servicemanager/DarknessService.cpp
    servicemanager/DarknessServiceFactory.h
    servicemanager/DarknessService.h
    servicemanager/DarknessServiceManager.cpp
    servicemanager/DarknessServiceManager.h
)

add_library(DarknessBase OBJECT ${DARKNESS_BASE_SOURCES})

target_include_directories(DarknessBase PUBLIC
    ${DARKNESS_BINARY_DIR}
    ${DARKNESS_SOURCE_DIR}/src/base
    ${DARKNESS_SOURCE_DIR}/src/base/console
    ${DARKNESS_SOURCE_DIR}/src/base/compat
    ${DARKNESS_SOURCE_DIR}/src/base/dyntype
    ${DARKNESS_SOURCE_DIR}/src/base/file
    ${DARKNESS_SOURCE_DIR}/src/base/loaders
    ${DARKNESS_SOURCE_DIR}/src/base/logger
    ${DARKNESS_SOURCE_DIR}/src/base/servicemanager
    ${DARKNESS_SOURCE_DIR}/src/compat
)

target_link_libraries(DarknessBase PUBLIC glm::glm)

target_compile_options(DarknessBase PRIVATE
    $<$<OR:$<CXX_COMPILER_ID:GNU>,$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:AppleClang>>:
        -Wall -fno-strict-aliasing -fvisibility=default>
)
