# Some global configuration checks

include(CheckIncludeFiles)

check_include_files(inttypes.h HAVE_INTTYPES_H)
mark_as_advanced(HAVE_INTTYPES_H)

# endianness check
include(TestBigEndian)
test_big_endian(BIG_ENDIAN)
mark_as_advanced(BIG_ENDIAN)

# TODO:
# Check for int/float size (should be both 4 bytes)
# Check for inttypes in more detail
# presence of standard headers
