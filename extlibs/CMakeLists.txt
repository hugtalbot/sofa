cmake_minimum_required(VERSION 3.1)

set(EXTLIBS_TARGETS
    csparse
    newmat
    tinyxml
)

######## SofaKernel/extlibs ########

# Nlohmann JSON (header-only): see SofaKernel/cmake/Modules/FindJson.cmake
set(JSON_ROOT "${SOFA_KERNEL_SOURCE_DIR}/extlibs/json" CACHE PATH "Nlohmann JSON directory")

# Newmat
find_package(Newmat QUIET)
if(NOT Newmat_FOUND)
    add_subdirectory(${SOFA_KERNEL_SOURCE_DIR}/extlibs/newmat newmat)
endif()

# TinyXML
find_package(TinyXML QUIET)
if(NOT TinyXML_FOUND)
    add_subdirectory(${SOFA_KERNEL_SOURCE_DIR}/extlibs/tinyxml tinyxml)
endif()

####################################

## Necessary ones

find_package(CSparse QUIET)
if(NOT CSparse_FOUND)
    add_subdirectory(csparse)
endif()

option(SOFA_BUILD_MINIFLOWVR "Build the miniFlowVR library distributed in the extlibs/ directory." OFF)
if(SOFA_BUILD_MINIFLOWVR)
    add_subdirectory(miniFlowVR)
    list(APPEND EXTLIBS_TARGETS miniFlowVR)
endif()

option(SOFA_BUILD_METIS
"Build the metis library distributed in the extlibs/ directory.  It is used only
by the SparseLDLSolver component.  Note that this library is usually available
on Linux distributions." OFF)
if(SOFA_BUILD_METIS)
    add_subdirectory(metis-5.1.0 "metis")
    list(APPEND EXTLIBS_TARGETS metis)
endif()

foreach(target ${EXTLIBS_TARGETS})
    set_target_properties(${target} PROPERTIES DEBUG_POSTFIX "_d")
    set_target_properties(${target} PROPERTIES FOLDER "SofaExtlibs")
endforeach()

