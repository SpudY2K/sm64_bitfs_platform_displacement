cmake_minimum_required(VERSION 3.15)
project(sm64_bitfs_platform_displacement
  LANGUAGES CXX
  DESCRIPTION "A bruteforcer that looks for combinations of platform angle, Mario's speed, and Mario's position that will upwarp him past the pole."
)

# Set default build type to Release.
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()

add_executable(bruteforce
  main.cpp
  vmath.cpp
  vmath.hpp
  Validate.cpp
  Validate.hpp
  Trig.cpp
  Trig.hpp
  Surface.cpp
  Surface.hpp
  Solution.hpp
  SearchUtils.cpp
  SearchUtils.hpp
  SearchParams.hpp
  Search.cpp
  Search.hpp
  Platform.cpp
  Platform.hpp
  Mario.cpp
  Mario.hpp
  Magic.cpp
  Magic.hpp
  Constants.hpp
)

# Equivalent to specifying a bunch of compiler flags.
# ===================================================

# LTO
include(CheckIPOSupported)
check_ipo_supported(RESULT lto_supported OUTPUT lto_error LANGUAGES CXX)
if(lto_supported)
  set_target_properties(bruteforce PROPERTIES
    INTERPROCEDURAL_OPTIMIZATION true
  )
endif()


# OpenMP
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
  target_link_libraries(bruteforce PRIVATE OpenMP::OpenMP_CXX)
endif()

# -march=native with MSVC emulation
# include("cmake/march_native.cmake")
# set_march_native(bruteforce)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-march=native" COMPILER_SUPPORTS_MARCH_NATIVE)
if(COMPILER_SUPPORTS_MARCH_NATIVE)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")
endif()
set(CMAKE_BUILD_TYPE RelWithDebInfo)

# -O3 is already handled by using release mode.