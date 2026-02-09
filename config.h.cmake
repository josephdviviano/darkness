#ifndef _CONFIG_H
#define _CONFIG_H
	
#cmakedefine HAVE_INTTYPES_H 1

#cmakedefine DARKNESS_DEBUG

#cmakedefine FRAME_PROFILER

#define DARKNESS_VER_MAJOR ${DARKNESS_VER_MAJOR}
#define DARKNESS_VER_MINOR ${DARKNESS_VER_MINOR}
#define DARKNESS_VER_PATCH ${DARKNESS_VER_PATCH}
#define DARKNESS_CODE_NAME "${REL_CODE_NAME}"

#cmakedefine __DARKNESS_BIG_ENDIAN ${BIG_ENDIAN}

#if defined (_MSC_VER)
// disable the class needs to have a dll-interface...
#pragma warning(disable:4251)
// No suitable definition for explicit template spec warning disable
#pragma warning(disable:4661)
#endif

// platforms
#cmakedefine WIN32
#cmakedefine UNIX
#cmakedefine APPLE

// data installation path
#define DARKNESS_SHARE_DIR "${DARKNESS_FULL_DATA_INSTALL_DIR}"

#endif
