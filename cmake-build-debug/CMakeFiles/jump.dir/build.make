# CMAKE generated file: DO NOT EDIT!
# Generated by "MinGW Makefiles" Generator, CMake Version 3.17

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Disable VCS-based implicit rules.
% : %,v


# Disable VCS-based implicit rules.
% : RCS/%


# Disable VCS-based implicit rules.
% : RCS/%,v


# Disable VCS-based implicit rules.
% : SCCS/s.%


# Disable VCS-based implicit rules.
% : s.%


.SUFFIXES: .hpux_make_needs_suffix_list


# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

SHELL = cmd.exe

# The CMake executable.
CMAKE_COMMAND = "C:\Program Files\JetBrains\CLion 2020.2\bin\cmake\win\bin\cmake.exe"

# The command to remove a file.
RM = "C:\Program Files\JetBrains\CLion 2020.2\bin\cmake\win\bin\cmake.exe" -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = M:\unitree_legged_sdk

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = M:\unitree_legged_sdk\cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/jump.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/jump.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/jump.dir/flags.make

CMakeFiles/jump.dir/examples/jump.cpp.obj: CMakeFiles/jump.dir/flags.make
CMakeFiles/jump.dir/examples/jump.cpp.obj: CMakeFiles/jump.dir/includes_CXX.rsp
CMakeFiles/jump.dir/examples/jump.cpp.obj: ../examples/jump.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=M:\unitree_legged_sdk\cmake-build-debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/jump.dir/examples/jump.cpp.obj"
	C:\MinGW\bin\g++.exe  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles\jump.dir\examples\jump.cpp.obj -c M:\unitree_legged_sdk\examples\jump.cpp

CMakeFiles/jump.dir/examples/jump.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/jump.dir/examples/jump.cpp.i"
	C:\MinGW\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E M:\unitree_legged_sdk\examples\jump.cpp > CMakeFiles\jump.dir\examples\jump.cpp.i

CMakeFiles/jump.dir/examples/jump.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/jump.dir/examples/jump.cpp.s"
	C:\MinGW\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S M:\unitree_legged_sdk\examples\jump.cpp -o CMakeFiles\jump.dir\examples\jump.cpp.s

# Object files for target jump
jump_OBJECTS = \
"CMakeFiles/jump.dir/examples/jump.cpp.obj"

# External object files for target jump
jump_EXTERNAL_OBJECTS =

jump.exe: CMakeFiles/jump.dir/examples/jump.cpp.obj
jump.exe: CMakeFiles/jump.dir/build.make
jump.exe: CMakeFiles/jump.dir/linklibs.rsp
jump.exe: CMakeFiles/jump.dir/objects1.rsp
jump.exe: CMakeFiles/jump.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=M:\unitree_legged_sdk\cmake-build-debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable jump.exe"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles\jump.dir\link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/jump.dir/build: jump.exe

.PHONY : CMakeFiles/jump.dir/build

CMakeFiles/jump.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles\jump.dir\cmake_clean.cmake
.PHONY : CMakeFiles/jump.dir/clean

CMakeFiles/jump.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" M:\unitree_legged_sdk M:\unitree_legged_sdk M:\unitree_legged_sdk\cmake-build-debug M:\unitree_legged_sdk\cmake-build-debug M:\unitree_legged_sdk\cmake-build-debug\CMakeFiles\jump.dir\DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/jump.dir/depend
