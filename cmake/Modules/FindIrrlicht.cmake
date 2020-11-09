## Based on https://github.com/facebookarchive/RakNet/blob/master/CmakeIncludes/FindIrrlicht.cmake

# - Try to find Irrlicht
# Once done this will define
#
#  IRRLICHT_FOUND - system has Irrlicht
#  IRRLICHT_INCLUDE_DIRS - the Irrlicht include directory
#  IRRLICHT_LIBRARIES - Link these to use Irrlicht
#  IRRLICHT_DEFINITIONS - Compiler switches required for using Irrlicht
#
#  Copyright (c) 2006 Andreas Schneider <mail@cynapses.org>
#
#  Redistribution and use is allowed according to the terms of the New
#  BSD license.
#  For details see the accompanying COPYING-CMAKE-SCRIPTS file.

## Edited by Simon Perche
# Use IRRLICHT_DIR environment variable

if (IRRLICHT_LIBRARIES AND IRRLICHT_INCLUDE_DIRS)
    # in cache already
    set(IRRLICHT_FOUND TRUE)
else (IRRLICHT_LIBRARIES AND IRRLICHT_INCLUDE_DIRS)
    find_path(IRRLICHT_INCLUDE_DIR
            NAMES
            irrlicht.h
            PATHS
            $ENV{IRRLICHT_DIR}/include
            )

    # If cmake found a visual studio 64-bit target
    if(${CMAKE_CL_64})
    find_library(IRRLICHT_LIBRARY
            NAMES
            Irrlicht
            PATHS
            $ENV{IRRLICHT_DIR}/lib/Win64-visualStudio
            )
    endif(${CMAKE_CL_64})

    if (IRRLICHT_LIBRARY)
        set(IRRLICHT_FOUND TRUE)
    endif (IRRLICHT_LIBRARY)

    set(IRRLICHT_INCLUDE_DIRS
            ${IRRLICHT_INCLUDE_DIR}
            )

    if (IRRLICHT_FOUND)
        set(IRRLICHT_LIBRARIES
                ${IRRLICHT_LIBRARIES}
                ${IRRLICHT_LIBRARY}
                )
    endif (IRRLICHT_FOUND)

    if (IRRLICHT_INCLUDE_DIRS AND IRRLICHT_LIBRARIES)
        set(IRRLICHT_FOUND TRUE)
    endif (IRRLICHT_INCLUDE_DIRS AND IRRLICHT_LIBRARIES)

    if (IRRLICHT_FOUND)
        if (NOT IRRLICHT_FIND_QUIETLY)
            message(STATUS "Found Irrlicht: ${IRRLICHT_LIBRARIES}")
        endif (NOT IRRLICHT_FIND_QUIETLY)
    else (IRRLICHT_FOUND)
        if (IRRLICHT_FIND_REQUIRED)
            message(FATAL_ERROR "Could not find Irrlicht")
        endif (IRRLICHT_FIND_REQUIRED)
    endif (IRRLICHT_FOUND)

    # show the IRRLICHT_INCLUDE_DIRS and IRRLICHT_LIBRARIES variables only in the advanced view
    mark_as_advanced(IRRLICHT_INCLUDE_DIRS IRRLICHT_LIBRARIES)

endif (IRRLICHT_LIBRARIES AND IRRLICHT_INCLUDE_DIRS)