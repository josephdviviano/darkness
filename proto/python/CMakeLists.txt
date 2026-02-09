INCLUDE( ${DARKNESS_SOURCE_DIR}/src/services/Services.cmake )

include_directories(
    ${ODE_INCLUDE_DIR}
    ${OGRE_INCLUDE_DIR}
    ${DARKNESS_BINARY_DIR}
    ${DARKNESS_BASE_INCLUDES}
    ${DARKNESS_SERVICE_INCLUDES}
    ${PYTHON_INCLUDE_PATH}
    ${DARKNESS_SOURCE_DIR}/src/bindings
    ${DARKNESS_SOURCE_DIR}/src/scenemanager
    ${DARKNESS_SOURCE_DIR}/src/main
    ${INCLUDE_ADDITION_CUSTOM_IMAGE}
)

add_executable(pytest
    test.cpp
    test.h
)

target_link_libraries(pytest
    $<TARGET_OBJECTS:OpenDark>
    $<TARGET_OBJECTS:OpenDarkPython>
    ${DARKNESS_PYTHON_LIBRARIES}
    ${OGRE_LIBRARIES}
)

SET_TARGET_PROPERTIES( pytest PROPERTIES
  COMPILE_DEFINITIONS "DARKNESS_EXE_TARGET"
)
