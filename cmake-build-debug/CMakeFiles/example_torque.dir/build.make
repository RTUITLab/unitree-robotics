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
include CMakeFiles/example_torque.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/example_torque.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/example_torque.dir/flags.make

CMakeFiles/example_torque.dir/examples/example_torque.cpp.obj: CMakeFiles/example_torque.dir/flags.make
CMakeFiles/example_torque.dir/examples/example_torque.cpp.obj: CMakeFiles/example_torque.dir/includes_CXX.rsp
CMakeFiles/example_torque.dir/examples/example_torque.cpp.obj: ../examples/example_torque.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=M:\unitree_legged_sdk\cmake-build-debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/example_torque.dir/examples/example_torque.cpp.obj"
	C:\MinGW\bin\g++.exe  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles\example_torque.dir\examples\example_torque.cpp.obj -c M:\unitree_legged_sdk\examples\example_torque.cpp

CMakeFiles/example_torque.dir/examples/example_torque.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/example_torque.dir/examples/example_torque.cpp.i"
	C:\MinGW\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E M:\unitree_legged_sdk\examples\example_torque.cpp > CMakeFiles\example_torque.dir\examples\example_torque.cpp.i

CMakeFiles/example_torque.dir/examples/example_torque.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/example_torque.dir/examples/example_torque.cpp.s"
	C:\MinGW\bin\g++.exe $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S M:\unitree_legged_sdk\examples\example_torque.cpp -o CMakeFiles\example_torque.dir\examples\example_torque.cpp.s

# Object files for target example_torque
example_torque_OBJECTS = \
"CMakeFiles/example_torque.dir/examples/example_torque.cpp.obj"

# External object files for target example_torque
example_torque_EXTERNAL_OBJECTS =

example_torque.exe: CMakeFiles/example_torque.dir/examples/example_torque.cpp.obj
example_torque.exe: CMakeFiles/example_torque.dir/build.make
example_torque.exe: CMakeFiles/example_torque.dir/linklibs.rsp
example_torque.exe: CMakeFiles/example_torque.dir/objects1.rsp
example_torque.exe: CMakeFiles/example_torque.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=M:\unitree_legged_sdk\cmake-build-debug\CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable example_torque.exe"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles\example_torque.dir\link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/example_torque.dir/build: example_torque.exe

.PHONY : CMakeFiles/example_torque.dir/build

CMakeFiles/example_torque.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles\example_torque.dir\cmake_clean.cmake
.PHONY : CMakeFiles/example_torque.dir/clean

CMakeFiles/example_torque.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" M:\unitree_legged_sdk M:\unitree_legged_sdk M:\unitree_legged_sdk\cmake-build-debug M:\unitree_legged_sdk\cmake-build-debug M:\unitree_legged_sdk\cmake-build-debug\CMakeFiles\example_torque.dir\DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/example_torque.dir/depend

