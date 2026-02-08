add_executable(readphs readphs.cpp)

target_include_directories(readphs PRIVATE
    ${OPDE_BINARY_DIR}
    ${OPDE_SOURCE_DIR}/src/base/compat
)
