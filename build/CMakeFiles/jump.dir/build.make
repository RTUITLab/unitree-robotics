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
CMAKE_SOURCE_DIR = /home/unitree/unitree_legged_sdk

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/unitree/unitree_legged_sdk/build

# Include any dependencies generated for this target.
include CMakeFiles/jump.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/jump.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/jump.dir/flags.make

CMakeFiles/jump.dir/examples/jump.cpp.o: CMakeFiles/jump.dir/flags.make
CMakeFiles/jump.dir/examples/jump.cpp.o: ../examples/jump.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/unitree/unitree_legged_sdk/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/jump.dir/examples/jump.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/jump.dir/examples/jump.cpp.o -c /home/unitree/unitree_legged_sdk/examples/jump.cpp

CMakeFiles/jump.dir/examples/jump.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/jump.dir/examples/jump.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/unitree/unitree_legged_sdk/examples/jump.cpp > CMakeFiles/jump.dir/examples/jump.cpp.i

CMakeFiles/jump.dir/examples/jump.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/jump.dir/examples/jump.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/unitree/unitree_legged_sdk/examples/jump.cpp -o CMakeFiles/jump.dir/examples/jump.cpp.s

CMakeFiles/jump.dir/examples/jump.cpp.o.requires:

.PHONY : CMakeFiles/jump.dir/examples/jump.cpp.o.requires

CMakeFiles/jump.dir/examples/jump.cpp.o.provides: CMakeFiles/jump.dir/examples/jump.cpp.o.requires
	$(MAKE) -f CMakeFiles/jump.dir/build.make CMakeFiles/jump.dir/examples/jump.cpp.o.provides.build
.PHONY : CMakeFiles/jump.dir/examples/jump.cpp.o.provides

CMakeFiles/jump.dir/examples/jump.cpp.o.provides.build: CMakeFiles/jump.dir/examples/jump.cpp.o


# Object files for target jump
jump_OBJECTS = \
"CMakeFiles/jump.dir/examples/jump.cpp.o"

# External object files for target jump
jump_EXTERNAL_OBJECTS =

jump: CMakeFiles/jump.dir/examples/jump.cpp.o
jump: CMakeFiles/jump.dir/build.make
jump: CMakeFiles/jump.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/unitree/unitree_legged_sdk/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable jump"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/jump.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/jump.dir/build: jump

.PHONY : CMakeFiles/jump.dir/build

CMakeFiles/jump.dir/requires: CMakeFiles/jump.dir/examples/jump.cpp.o.requires

.PHONY : CMakeFiles/jump.dir/requires

CMakeFiles/jump.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/jump.dir/cmake_clean.cmake
.PHONY : CMakeFiles/jump.dir/clean

CMakeFiles/jump.dir/depend:
	cd /home/unitree/unitree_legged_sdk/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/unitree/unitree_legged_sdk /home/unitree/unitree_legged_sdk /home/unitree/unitree_legged_sdk/build /home/unitree/unitree_legged_sdk/build /home/unitree/unitree_legged_sdk/build/CMakeFiles/jump.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/jump.dir/depend
