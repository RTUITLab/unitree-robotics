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
include CMakeFiles/sandbox.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/sandbox.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/sandbox.dir/flags.make

CMakeFiles/sandbox.dir/examples/sandbox.cpp.o: CMakeFiles/sandbox.dir/flags.make
CMakeFiles/sandbox.dir/examples/sandbox.cpp.o: ../examples/sandbox.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/unitree/unitree_legged_sdk/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/sandbox.dir/examples/sandbox.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sandbox.dir/examples/sandbox.cpp.o -c /home/unitree/unitree_legged_sdk/examples/sandbox.cpp

CMakeFiles/sandbox.dir/examples/sandbox.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sandbox.dir/examples/sandbox.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/unitree/unitree_legged_sdk/examples/sandbox.cpp > CMakeFiles/sandbox.dir/examples/sandbox.cpp.i

CMakeFiles/sandbox.dir/examples/sandbox.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sandbox.dir/examples/sandbox.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/unitree/unitree_legged_sdk/examples/sandbox.cpp -o CMakeFiles/sandbox.dir/examples/sandbox.cpp.s

CMakeFiles/sandbox.dir/examples/sandbox.cpp.o.requires:

.PHONY : CMakeFiles/sandbox.dir/examples/sandbox.cpp.o.requires

CMakeFiles/sandbox.dir/examples/sandbox.cpp.o.provides: CMakeFiles/sandbox.dir/examples/sandbox.cpp.o.requires
	$(MAKE) -f CMakeFiles/sandbox.dir/build.make CMakeFiles/sandbox.dir/examples/sandbox.cpp.o.provides.build
.PHONY : CMakeFiles/sandbox.dir/examples/sandbox.cpp.o.provides

CMakeFiles/sandbox.dir/examples/sandbox.cpp.o.provides.build: CMakeFiles/sandbox.dir/examples/sandbox.cpp.o


# Object files for target sandbox
sandbox_OBJECTS = \
"CMakeFiles/sandbox.dir/examples/sandbox.cpp.o"

# External object files for target sandbox
sandbox_EXTERNAL_OBJECTS =

sandbox: CMakeFiles/sandbox.dir/examples/sandbox.cpp.o
sandbox: CMakeFiles/sandbox.dir/build.make
sandbox: CMakeFiles/sandbox.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/unitree/unitree_legged_sdk/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable sandbox"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/sandbox.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/sandbox.dir/build: sandbox

.PHONY : CMakeFiles/sandbox.dir/build

CMakeFiles/sandbox.dir/requires: CMakeFiles/sandbox.dir/examples/sandbox.cpp.o.requires

.PHONY : CMakeFiles/sandbox.dir/requires

CMakeFiles/sandbox.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/sandbox.dir/cmake_clean.cmake
.PHONY : CMakeFiles/sandbox.dir/clean

CMakeFiles/sandbox.dir/depend:
	cd /home/unitree/unitree_legged_sdk/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/unitree/unitree_legged_sdk /home/unitree/unitree_legged_sdk /home/unitree/unitree_legged_sdk/build /home/unitree/unitree_legged_sdk/build /home/unitree/unitree_legged_sdk/build/CMakeFiles/sandbox.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/sandbox.dir/depend

