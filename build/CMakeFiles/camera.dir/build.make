# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_SOURCE_DIR = /home/wangduo/Code/ceilingLocalize

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/wangduo/Code/ceilingLocalize/build

# Include any dependencies generated for this target.
include CMakeFiles/camera.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/camera.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/camera.dir/flags.make

CMakeFiles/camera.dir/src/camera.cpp.o: CMakeFiles/camera.dir/flags.make
CMakeFiles/camera.dir/src/camera.cpp.o: ../src/camera.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/wangduo/Code/ceilingLocalize/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/camera.dir/src/camera.cpp.o"
	g++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/camera.dir/src/camera.cpp.o -c /home/wangduo/Code/ceilingLocalize/src/camera.cpp

CMakeFiles/camera.dir/src/camera.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/camera.dir/src/camera.cpp.i"
	g++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/wangduo/Code/ceilingLocalize/src/camera.cpp > CMakeFiles/camera.dir/src/camera.cpp.i

CMakeFiles/camera.dir/src/camera.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/camera.dir/src/camera.cpp.s"
	g++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/wangduo/Code/ceilingLocalize/src/camera.cpp -o CMakeFiles/camera.dir/src/camera.cpp.s

CMakeFiles/camera.dir/src/camera.cpp.o.requires:
.PHONY : CMakeFiles/camera.dir/src/camera.cpp.o.requires

CMakeFiles/camera.dir/src/camera.cpp.o.provides: CMakeFiles/camera.dir/src/camera.cpp.o.requires
	$(MAKE) -f CMakeFiles/camera.dir/build.make CMakeFiles/camera.dir/src/camera.cpp.o.provides.build
.PHONY : CMakeFiles/camera.dir/src/camera.cpp.o.provides

CMakeFiles/camera.dir/src/camera.cpp.o.provides.build: CMakeFiles/camera.dir/src/camera.cpp.o

# Object files for target camera
camera_OBJECTS = \
"CMakeFiles/camera.dir/src/camera.cpp.o"

# External object files for target camera
camera_EXTERNAL_OBJECTS =

../lib/libcamera.a: CMakeFiles/camera.dir/src/camera.cpp.o
../lib/libcamera.a: CMakeFiles/camera.dir/build.make
../lib/libcamera.a: CMakeFiles/camera.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX static library ../lib/libcamera.a"
	$(CMAKE_COMMAND) -P CMakeFiles/camera.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/camera.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/camera.dir/build: ../lib/libcamera.a
.PHONY : CMakeFiles/camera.dir/build

CMakeFiles/camera.dir/requires: CMakeFiles/camera.dir/src/camera.cpp.o.requires
.PHONY : CMakeFiles/camera.dir/requires

CMakeFiles/camera.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/camera.dir/cmake_clean.cmake
.PHONY : CMakeFiles/camera.dir/clean

CMakeFiles/camera.dir/depend:
	cd /home/wangduo/Code/ceilingLocalize/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/wangduo/Code/ceilingLocalize /home/wangduo/Code/ceilingLocalize /home/wangduo/Code/ceilingLocalize/build /home/wangduo/Code/ceilingLocalize/build /home/wangduo/Code/ceilingLocalize/build/CMakeFiles/camera.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/camera.dir/depend

