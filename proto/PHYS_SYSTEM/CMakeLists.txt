add_executable(readphs readphs.cpp)

target_include_directories(readphs PRIVATE
    ${DARKNESS_BINARY_DIR}
    ${DARKNESS_SOURCE_DIR}/src/base/compat
)
