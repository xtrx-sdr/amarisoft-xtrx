cmake_minimum_required(VERSION 2.6)
project(amarisoft_xtrx C)
enable_testing()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/Modules)

find_package(LibXTRX)

# Set the version information here
set(MAJOR_VERSION 0)
set(API_COMPAT    0)
set(MINOR_VERSION 1)
set(MAINT_VERSION git)

set(LIBVER  "${MAJOR_VERSION}.${API_COMPAT}.${MINOR_VERSION}")

set(AMARISOFT_EXTRA_PATH "${CMAKE_CURRENT_SOURCE_DIR}" CACHE STRING "Path for searching trx_driver.h")
message(" --- ${AMARISOFT_EXTRA_PATH} ")
find_path(AMARISOFT_LTEENB_INCLUDE_DIRS NAMES trx_driver.h
    PATHS
    ${AMARISOFT_EXTRA_PATH}
    /usr/include
    /usr/local/include
    /opt/amarisoft/include
)

if(AMARISOFT_LTEENB_INCLUDE_DIRS)
    message(STATUS "Amarisoft trx_driver.h path: ${AMARISOFT_LTEENB_INCLUDE_DIRS}")
else(AMARISOFT_LTEENB_INCLUDE_DIRS)
    message(FATAL_ERROR "Amarisoft trx_driver.h not found")
endif(AMARISOFT_LTEENB_INCLUDE_DIRS)

include_directories(${AMARISOFT_LTEENB_INCLUDE_DIRS})
add_definitions(-Werror -Wall -g)

set(BUILD_SHARED_LIBS ON)

add_library(trx_xtrx SHARED xtrx_plugin.c)
target_link_libraries(trx_xtrx xtrx pthread rt)
