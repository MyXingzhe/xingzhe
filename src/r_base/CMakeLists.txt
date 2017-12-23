#-----------------------------------------------------------------------------------------------
#
#  Run with "ccmake ." at the root directory, or use it as a template for 
#   starting your own programs
#-----------------------------------------------------------------------------------------------
SET(projectName R_BASE)
SET(PRJ_NAME "EXAMPLE_${projectName}")

# ---------------------------------------
# Declare a new CMake Project:
# ---------------------------------------
PROJECT(${PRJ_NAME})

CMAKE_MINIMUM_REQUIRED(VERSION 3.1)
set (CMAKE_CXX_STANDARD 14)  # Require C++14
cmake_policy(SET CMP0003 NEW)  # Required by CMake 2.7+
#cmake_policy(SET CMP0043 OLD) #  Ignore COMPILE_DEFINITIONS_<Config> properties.

# ---------------------------------------------------------------------------
# Set the output directory of each example to its corresponding subdirectory
#  in the binary tree:
# ---------------------------------------------------------------------------
SET(EXECUTABLE_OUTPUT_PATH ".")

# --------------------------------------------------------------------------
#   The list of "libs" which can be included can be found in:
#     http://www.mrpt.org/Libraries
#
#   The dependencies of a library are automatically added, so you only 
#    need to specify the top-most libraries your code depend on.
# --------------------------------------------------------------------------
FIND_PACKAGE(MRPT REQUIRED base;bayes;obs;gui)

# ---------------------------------------------
# TARGET:
# ---------------------------------------------
# Define the executable target:
ADD_EXECUTABLE(r_base src/gui.cpp  ) 

SET_TARGET_PROPERTIES(
	r_base 
	PROPERTIES 
	PROJECT_LABEL "(EXAMPLE) ${projectName}")

# Add special defines needed by this example, if any:
SET(MY_DEFS )
IF(MY_DEFS) # If not empty
	ADD_DEFINITIONS("-D${MY_DEFS}")
ENDIF(MY_DEFS)

# Add the required libraries for linking:
TARGET_LINK_LIBRARIES(r_base 
	${MRPT_LIBS}  # This is filled by FIND_PACKAGE(MRPT ...)
	""  # Optional extra libs...
	)

#LINK_DIRECTORIES(/usr/local/lib/)
add_compile_options("`wx-config --cxxflags --unicode=yes --static=no --toolkit=gtk3 --version=3.0`")

# Set optimized building:
IF((${CMAKE_CXX_COMPILER_ID} STREQUAL "Clang" OR CMAKE_COMPILER_IS_GNUCXX) AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
	add_compile_options(-O3)
ENDIF()

#DeclareAppDependencies(${projectName} mrpt-base;mrpt-bayes;mrpt-obs;mrpt-gui) # Dependencies
# -------------------------------------------------------------------------

