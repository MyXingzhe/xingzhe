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
CMAKE_SOURCE_DIR = /home/ubuntu/projects/xingzhe/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ubuntu/projects/xingzhe/src

# Utility rule file for prupwm_geneus.

# Include the progress variables for this target.
include prupwm/CMakeFiles/prupwm_geneus.dir/progress.make

prupwm_geneus: prupwm/CMakeFiles/prupwm_geneus.dir/build.make

.PHONY : prupwm_geneus

# Rule to build all files generated by this target.
prupwm/CMakeFiles/prupwm_geneus.dir/build: prupwm_geneus

.PHONY : prupwm/CMakeFiles/prupwm_geneus.dir/build

prupwm/CMakeFiles/prupwm_geneus.dir/clean:
	cd /home/ubuntu/projects/xingzhe/src/prupwm && $(CMAKE_COMMAND) -P CMakeFiles/prupwm_geneus.dir/cmake_clean.cmake
.PHONY : prupwm/CMakeFiles/prupwm_geneus.dir/clean

prupwm/CMakeFiles/prupwm_geneus.dir/depend:
	cd /home/ubuntu/projects/xingzhe/src && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ubuntu/projects/xingzhe/src /home/ubuntu/projects/xingzhe/src/prupwm /home/ubuntu/projects/xingzhe/src /home/ubuntu/projects/xingzhe/src/prupwm /home/ubuntu/projects/xingzhe/src/prupwm/CMakeFiles/prupwm_geneus.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : prupwm/CMakeFiles/prupwm_geneus.dir/depend

