# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/bone/projects/xingzhe/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/bone/projects/xingzhe/src

# Utility rule file for ear_geneus.

# Include the progress variables for this target.
include ear/CMakeFiles/ear_geneus.dir/progress.make

ear_geneus: ear/CMakeFiles/ear_geneus.dir/build.make

.PHONY : ear_geneus

# Rule to build all files generated by this target.
ear/CMakeFiles/ear_geneus.dir/build: ear_geneus

.PHONY : ear/CMakeFiles/ear_geneus.dir/build

ear/CMakeFiles/ear_geneus.dir/clean:
	cd /home/bone/projects/xingzhe/src/ear && $(CMAKE_COMMAND) -P CMakeFiles/ear_geneus.dir/cmake_clean.cmake
.PHONY : ear/CMakeFiles/ear_geneus.dir/clean

ear/CMakeFiles/ear_geneus.dir/depend:
	cd /home/bone/projects/xingzhe/src && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/bone/projects/xingzhe/src /home/bone/projects/xingzhe/src/ear /home/bone/projects/xingzhe/src /home/bone/projects/xingzhe/src/ear /home/bone/projects/xingzhe/src/ear/CMakeFiles/ear_geneus.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : ear/CMakeFiles/ear_geneus.dir/depend

