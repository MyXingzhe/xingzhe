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
include body/CMakeFiles/body_node.dir/depend.make

# Include the progress variables for this target.
include body/CMakeFiles/body_node.dir/progress.make

# Include the compile flags for this target's objects.
include body/CMakeFiles/body_node.dir/flags.make

body/CMakeFiles/body_node.dir/src/body_node.cpp.o: body/CMakeFiles/body_node.dir/flags.make
body/CMakeFiles/body_node.dir/src/body_node.cpp.o: body/src/body_node.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/bbb/projects/xingzhe/src/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object body/CMakeFiles/body_node.dir/src/body_node.cpp.o"
	cd /home/bbb/projects/xingzhe/src/body && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/body_node.dir/src/body_node.cpp.o -c /home/bbb/projects/xingzhe/src/body/src/body_node.cpp

body/CMakeFiles/body_node.dir/src/body_node.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/body_node.dir/src/body_node.cpp.i"
	cd /home/bbb/projects/xingzhe/src/body && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/bbb/projects/xingzhe/src/body/src/body_node.cpp > CMakeFiles/body_node.dir/src/body_node.cpp.i

body/CMakeFiles/body_node.dir/src/body_node.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/body_node.dir/src/body_node.cpp.s"
	cd /home/bbb/projects/xingzhe/src/body && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/bbb/projects/xingzhe/src/body/src/body_node.cpp -o CMakeFiles/body_node.dir/src/body_node.cpp.s

body/CMakeFiles/body_node.dir/src/body_node.cpp.o.requires:

.PHONY : body/CMakeFiles/body_node.dir/src/body_node.cpp.o.requires

body/CMakeFiles/body_node.dir/src/body_node.cpp.o.provides: body/CMakeFiles/body_node.dir/src/body_node.cpp.o.requires
	$(MAKE) -f body/CMakeFiles/body_node.dir/build.make body/CMakeFiles/body_node.dir/src/body_node.cpp.o.provides.build
.PHONY : body/CMakeFiles/body_node.dir/src/body_node.cpp.o.provides

body/CMakeFiles/body_node.dir/src/body_node.cpp.o.provides.build: body/CMakeFiles/body_node.dir/src/body_node.cpp.o


body/CMakeFiles/body_node.dir/src/body.cpp.o: body/CMakeFiles/body_node.dir/flags.make
body/CMakeFiles/body_node.dir/src/body.cpp.o: body/src/body.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/bbb/projects/xingzhe/src/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object body/CMakeFiles/body_node.dir/src/body.cpp.o"
	cd /home/bbb/projects/xingzhe/src/body && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/body_node.dir/src/body.cpp.o -c /home/bbb/projects/xingzhe/src/body/src/body.cpp

body/CMakeFiles/body_node.dir/src/body.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/body_node.dir/src/body.cpp.i"
	cd /home/bbb/projects/xingzhe/src/body && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/bbb/projects/xingzhe/src/body/src/body.cpp > CMakeFiles/body_node.dir/src/body.cpp.i

body/CMakeFiles/body_node.dir/src/body.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/body_node.dir/src/body.cpp.s"
	cd /home/bbb/projects/xingzhe/src/body && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/bbb/projects/xingzhe/src/body/src/body.cpp -o CMakeFiles/body_node.dir/src/body.cpp.s

body/CMakeFiles/body_node.dir/src/body.cpp.o.requires:

.PHONY : body/CMakeFiles/body_node.dir/src/body.cpp.o.requires

body/CMakeFiles/body_node.dir/src/body.cpp.o.provides: body/CMakeFiles/body_node.dir/src/body.cpp.o.requires
	$(MAKE) -f body/CMakeFiles/body_node.dir/build.make body/CMakeFiles/body_node.dir/src/body.cpp.o.provides.build
.PHONY : body/CMakeFiles/body_node.dir/src/body.cpp.o.provides

body/CMakeFiles/body_node.dir/src/body.cpp.o.provides.build: body/CMakeFiles/body_node.dir/src/body.cpp.o


body/CMakeFiles/body_node.dir/src/MPU6050.cpp.o: body/CMakeFiles/body_node.dir/flags.make
body/CMakeFiles/body_node.dir/src/MPU6050.cpp.o: body/src/MPU6050.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/bbb/projects/xingzhe/src/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object body/CMakeFiles/body_node.dir/src/MPU6050.cpp.o"
	cd /home/bbb/projects/xingzhe/src/body && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/body_node.dir/src/MPU6050.cpp.o -c /home/bbb/projects/xingzhe/src/body/src/MPU6050.cpp

body/CMakeFiles/body_node.dir/src/MPU6050.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/body_node.dir/src/MPU6050.cpp.i"
	cd /home/bbb/projects/xingzhe/src/body && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/bbb/projects/xingzhe/src/body/src/MPU6050.cpp > CMakeFiles/body_node.dir/src/MPU6050.cpp.i

body/CMakeFiles/body_node.dir/src/MPU6050.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/body_node.dir/src/MPU6050.cpp.s"
	cd /home/bbb/projects/xingzhe/src/body && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/bbb/projects/xingzhe/src/body/src/MPU6050.cpp -o CMakeFiles/body_node.dir/src/MPU6050.cpp.s

body/CMakeFiles/body_node.dir/src/MPU6050.cpp.o.requires:

.PHONY : body/CMakeFiles/body_node.dir/src/MPU6050.cpp.o.requires

body/CMakeFiles/body_node.dir/src/MPU6050.cpp.o.provides: body/CMakeFiles/body_node.dir/src/MPU6050.cpp.o.requires
	$(MAKE) -f body/CMakeFiles/body_node.dir/build.make body/CMakeFiles/body_node.dir/src/MPU6050.cpp.o.provides.build
.PHONY : body/CMakeFiles/body_node.dir/src/MPU6050.cpp.o.provides

body/CMakeFiles/body_node.dir/src/MPU6050.cpp.o.provides.build: body/CMakeFiles/body_node.dir/src/MPU6050.cpp.o


# Object files for target body_node
body_node_OBJECTS = \
"CMakeFiles/body_node.dir/src/body_node.cpp.o" \
"CMakeFiles/body_node.dir/src/body.cpp.o" \
"CMakeFiles/body_node.dir/src/MPU6050.cpp.o"

# External object files for target body_node
body_node_EXTERNAL_OBJECTS =

/home/bbb/projects/xingzhe/devel/lib/body/body_node: body/CMakeFiles/body_node.dir/src/body_node.cpp.o
/home/bbb/projects/xingzhe/devel/lib/body/body_node: body/CMakeFiles/body_node.dir/src/body.cpp.o
/home/bbb/projects/xingzhe/devel/lib/body/body_node: body/CMakeFiles/body_node.dir/src/MPU6050.cpp.o
/home/bbb/projects/xingzhe/devel/lib/body/body_node: body/CMakeFiles/body_node.dir/build.make
/home/bbb/projects/xingzhe/devel/lib/body/body_node: /opt/ros/kinetic/lib/libroscpp.so
/home/bbb/projects/xingzhe/devel/lib/body/body_node: /usr/lib/i386-linux-gnu/libboost_signals.so
/home/bbb/projects/xingzhe/devel/lib/body/body_node: /usr/lib/i386-linux-gnu/libboost_filesystem.so
/home/bbb/projects/xingzhe/devel/lib/body/body_node: /opt/ros/kinetic/lib/librosconsole.so
/home/bbb/projects/xingzhe/devel/lib/body/body_node: /opt/ros/kinetic/lib/librosconsole_log4cxx.so
/home/bbb/projects/xingzhe/devel/lib/body/body_node: /opt/ros/kinetic/lib/librosconsole_backend_interface.so
/home/bbb/projects/xingzhe/devel/lib/body/body_node: /usr/lib/i386-linux-gnu/liblog4cxx.so
/home/bbb/projects/xingzhe/devel/lib/body/body_node: /usr/lib/i386-linux-gnu/libboost_regex.so
/home/bbb/projects/xingzhe/devel/lib/body/body_node: /opt/ros/kinetic/lib/libxmlrpcpp.so
/home/bbb/projects/xingzhe/devel/lib/body/body_node: /opt/ros/kinetic/lib/libroscpp_serialization.so
/home/bbb/projects/xingzhe/devel/lib/body/body_node: /opt/ros/kinetic/lib/librostime.so
/home/bbb/projects/xingzhe/devel/lib/body/body_node: /opt/ros/kinetic/lib/libcpp_common.so
/home/bbb/projects/xingzhe/devel/lib/body/body_node: /usr/lib/i386-linux-gnu/libboost_system.so
/home/bbb/projects/xingzhe/devel/lib/body/body_node: /usr/lib/i386-linux-gnu/libboost_thread.so
/home/bbb/projects/xingzhe/devel/lib/body/body_node: /usr/lib/i386-linux-gnu/libboost_chrono.so
/home/bbb/projects/xingzhe/devel/lib/body/body_node: /usr/lib/i386-linux-gnu/libboost_date_time.so
/home/bbb/projects/xingzhe/devel/lib/body/body_node: /usr/lib/i386-linux-gnu/libboost_atomic.so
/home/bbb/projects/xingzhe/devel/lib/body/body_node: /usr/lib/i386-linux-gnu/libpthread.so
/home/bbb/projects/xingzhe/devel/lib/body/body_node: /usr/lib/i386-linux-gnu/libconsole_bridge.so
/home/bbb/projects/xingzhe/devel/lib/body/body_node: /home/bbb/projects/xingzhe/devel/lib/libi2c.so
/home/bbb/projects/xingzhe/devel/lib/body/body_node: body/CMakeFiles/body_node.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/bbb/projects/xingzhe/src/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable /home/bbb/projects/xingzhe/devel/lib/body/body_node"
	cd /home/bbb/projects/xingzhe/src/body && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/body_node.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
body/CMakeFiles/body_node.dir/build: /home/bbb/projects/xingzhe/devel/lib/body/body_node

.PHONY : body/CMakeFiles/body_node.dir/build

body/CMakeFiles/body_node.dir/requires: body/CMakeFiles/body_node.dir/src/body_node.cpp.o.requires
body/CMakeFiles/body_node.dir/requires: body/CMakeFiles/body_node.dir/src/body.cpp.o.requires
body/CMakeFiles/body_node.dir/requires: body/CMakeFiles/body_node.dir/src/MPU6050.cpp.o.requires

.PHONY : body/CMakeFiles/body_node.dir/requires

body/CMakeFiles/body_node.dir/clean:
	cd /home/bbb/projects/xingzhe/src/body && $(CMAKE_COMMAND) -P CMakeFiles/body_node.dir/cmake_clean.cmake
.PHONY : body/CMakeFiles/body_node.dir/clean

body/CMakeFiles/body_node.dir/depend:
	cd /home/bbb/projects/xingzhe/src && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/bbb/projects/xingzhe/src /home/bbb/projects/xingzhe/src/body /home/bbb/projects/xingzhe/src /home/bbb/projects/xingzhe/src/body /home/bbb/projects/xingzhe/src/body/CMakeFiles/body_node.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : body/CMakeFiles/body_node.dir/depend
