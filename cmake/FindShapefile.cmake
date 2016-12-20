# - Try to find Shapefile C Library
#   http://shapelib.maptools.org/
#
# Once done, this will define
#
#  Shapefile_FOUND - system has Shapefile
#  Shapefile_INCLUDE_DIRS - the Shapefile include directories
#  Shapefile_LIBRARIES - link these to use Shapefile

find_path(Shapefile_INCLUDE_DIR NAMES shapefil.h)
find_library(Shapefile_LIBRARY NAMES shp)

set(Shapefile_FOUND TRUE)
if ("${Shapefile_INCLUDE_DIR}" STREQUAL "Shapefile_INCLUDE_DIR-NOTFOUND" OR
    "${Shapefile_LIBRARY}" STREQUAL "Shapefile_LIBRARY-NOTFOUND")
  set(Shapefile_FOUND FALSE)
endif()
