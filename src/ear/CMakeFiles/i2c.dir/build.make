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
CMAKE_SOURCE_DIR = /home/bbb/projects/xingzhe/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/bbb/projects/xingzhe/src

# Include any dependencies generated for this target.
include ear/CMakeFiles/i2c.dir/depend.make

# Include the progress variables for this target.
include ear/CMakeFiles/i2c.dir/progress.make

# Include the compile flags for this target's objects.
include ear/CMakeFiles/i2c.dir/flags.make

ear/CMakeFiles/i2c.dir/__/i2c/i2c_dev.cpp.o: ear/CMakeFiles/i2c.dir/flags.make
ear/CMakeFiles/i2c.dir/__/i2c/i2c_dev.cpp.o: i2c/i2c_dev.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/bbb/projects/xingzhe/src/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object ear/CMakeFiles/i2c.dir/__/i2c/i2c_dev.cpp.o"
	cd /home/bbb/projects/xingzhe/src/ear && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/i2c.dir/__/i2c/i2c_dev.cpp.o -c /home/bbb/projects/xingzhe/src/i2c/i2c_dev.cpp

ear/CMakeFiles/i2c.dir/__/i2c/i2c_dev.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/i2c.dir/__/i2c/i2c_dev.cpp.i"
	cd /home/bbb/projects/xingzhe/src/ear && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/bbb/projects/xingzhe/src/i2c/i2c_dev.cpp > CMakeFiles/i2c.dir/__/i2c/i2c_dev.cpp.i

ear/CMakeFiles/i2c.dir/__/i2c/i2c_dev.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/i2c.dir/__/i2c/i2c_dev.cpp.s"
	cd /home/bbb/projects/xingzhe/src/ear && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/bbb/projects/xingzhe/src/i2c/i2c_dev.cpp -o CMakeFiles/i2c.dir/__/i2c/i2c_dev.cpp.s

ear/CMakeFiles/i2c.dir/__/i2c/i2c_dev.cpp.o.requires:

.PHONY : ear/CMakeFiles/i2c.dir/__/i2c/i2c_dev.cpp.o.requires

ear/CMakeFiles/i2c.dir/__/i2c/i2c_dev.cpp.o.provides: ear/CMakeFiles/i2c.dir/__/i2c/i2c_dev.cpp.o.requires
	$(MAKE) -f ear/CMakeFiles/i2c.dir/build.make ear/CMakeFiles/i2c.dir/__/i2c/i2c_dev.cpp.o.provides.build
.PHONY : ear/CMakeFiles/i2c.dir/__/i2c/i2c_dev.cpp.o.provides

ear/CMakeFiles/i2c.dir/__/i2c/i2c_dev.cpp.o.provides.build: ear/CMakeFiles/i2c.dir/__/i2c/i2c_dev.cpp.o


# Object files for target i2c
i2c_OBJECTS = \
"CMakeFiles/i2c.dir/__/i2c/i2c_dev.cpp.o"

# External object files for target i2c
i2c_EXTERNAL_OBJECTS =

/home/bbb/projects/xingzhe/devel/lib/libi2c.so: ear/CMakeFiles/i2c.dir/__/i2c/i2c_dev.cpp.o
/home/bbb/projects/xingzhe/devel/lib/libi2c.so: ear/CMakeFiles/i2c.dir/build.make
/home/bbb/projects/xingzhe/devel/lib/libi2c.so: ear/CMakeFiles/i2c.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/bbb/projects/xingzhe/src/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library /home/bbb/projects/xingzhe/devel/lib/libi2c.so"
	cd /home/bbb/projects/xingzhe/src/ear && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/i2c.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
ear/CMakeFiles/i2c.dir/build: /home/bbb/projects/xingzhe/devel/lib/libi2c.so

.PHONY : ear/CMakeFiles/i2c.dir/build

ear/CMakeFiles/i2c.dir/requires: ear/CMakeFiles/i2c.dir/__/i2c/i2c_dev.cpp.o.requires

.PHONY : ear/CMakeFiles/i2c.dir/requires

ear/CMakeFiles/i2c.dir/clean:
	cd /home/bbb/projects/xingzhe/src/ear && $(CMAKE_COMMAND) -P CMakeFiles/i2c.dir/cmake_clean.cmake
.PHONY : ear/CMakeFiles/i2c.dir/clean

ear/CMakeFiles/i2c.dir/depend:
	cd /home/bbb/projects/xingzhe/src && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/bbb/projects/xingzhe/src /home/bbb/projects/xingzhe/src/ear /home/bbb/projects/xingzhe/src /home/bbb/projects/xingzhe/src/ear /home/bbb/projects/xingzhe/src/ear/CMakeFiles/i2c.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : ear/CMakeFiles/i2c.dir/depend

