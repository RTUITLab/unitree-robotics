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
include CMakeFiles/universal.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/universal.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/universal.dir/flags.make

CMakeFiles/universal.dir/examples/universal.cpp.o: CMakeFiles/universal.dir/flags.make
CMakeFiles/universal.dir/examples/universal.cpp.o: ../examples/universal.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/unitree/unitree_legged_sdk/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/universal.dir/examples/universal.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/universal.dir/examples/universal.cpp.o -c /home/unitree/unitree_legged_sdk/examples/universal.cpp

CMakeFiles/universal.dir/examples/universal.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/universal.dir/examples/universal.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/unitree/unitree_legged_sdk/examples/universal.cpp > CMakeFiles/universal.dir/examples/universal.cpp.i

CMakeFiles/universal.dir/examples/universal.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/universal.dir/examples/universal.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/unitree/unitree_legged_sdk/examples/universal.cpp -o CMakeFiles/universal.dir/examples/universal.cpp.s

CMakeFiles/universal.dir/examples/universal.cpp.o.requires:

.PHONY : CMakeFiles/universal.dir/examples/universal.cpp.o.requires

CMakeFiles/universal.dir/examples/universal.cpp.o.provides: CMakeFiles/universal.dir/examples/universal.cpp.o.requires
	$(MAKE) -f CMakeFiles/universal.dir/build.make CMakeFiles/universal.dir/examples/universal.cpp.o.provides.build
.PHONY : CMakeFiles/universal.dir/examples/universal.cpp.o.provides

CMakeFiles/universal.dir/examples/universal.cpp.o.provides.build: CMakeFiles/universal.dir/examples/universal.cpp.o


# Object files for target universal
universal_OBJECTS = \
"CMakeFiles/universal.dir/examples/universal.cpp.o"

# External object files for target universal
universal_EXTERNAL_OBJECTS =

universal: CMakeFiles/universal.dir/examples/universal.cpp.o
universal: CMakeFiles/universal.dir/build.make
universal: CMakeFiles/universal.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/unitree/unitree_legged_sdk/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable universal"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/universal.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/universal.dir/build: universal

.PHONY : CMakeFiles/universal.dir/build

CMakeFiles/universal.dir/requires: CMakeFiles/universal.dir/examples/universal.cpp.o.requires

.PHONY : CMakeFiles/universal.dir/requires

CMakeFiles/universal.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/universal.dir/cmake_clean.cmake
.PHONY : CMakeFiles/universal.dir/clean

CMakeFiles/universal.dir/depend:
	cd /home/unitree/unitree_legged_sdk/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/unitree/unitree_legged_sdk /home/unitree/unitree_legged_sdk /home/unitree/unitree_legged_sdk/build /home/unitree/unitree_legged_sdk/build /home/unitree/unitree_legged_sdk/build/CMakeFiles/universal.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/universal.dir/depend
