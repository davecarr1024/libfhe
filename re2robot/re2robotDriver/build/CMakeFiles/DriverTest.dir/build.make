# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canoncical targets will work.
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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/dave/ros/re2robot/re2robotDriver

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/dave/ros/re2robot/re2robotDriver/build

# Include any dependencies generated for this target.
include CMakeFiles/DriverTest.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/DriverTest.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/DriverTest.dir/flags.make

CMakeFiles/DriverTest.dir/src/tests/DriverTest.o: CMakeFiles/DriverTest.dir/flags.make
CMakeFiles/DriverTest.dir/src/tests/DriverTest.o: ../src/tests/DriverTest.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/dave/ros/re2robot/re2robotDriver/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/DriverTest.dir/src/tests/DriverTest.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -DTIXML_USE_STL -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -o CMakeFiles/DriverTest.dir/src/tests/DriverTest.o -c /home/dave/ros/re2robot/re2robotDriver/src/tests/DriverTest.cpp

CMakeFiles/DriverTest.dir/src/tests/DriverTest.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/DriverTest.dir/src/tests/DriverTest.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -DTIXML_USE_STL -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -E /home/dave/ros/re2robot/re2robotDriver/src/tests/DriverTest.cpp > CMakeFiles/DriverTest.dir/src/tests/DriverTest.i

CMakeFiles/DriverTest.dir/src/tests/DriverTest.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/DriverTest.dir/src/tests/DriverTest.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -DTIXML_USE_STL -W -Wall -Wno-unused-parameter -fno-strict-aliasing -pthread -S /home/dave/ros/re2robot/re2robotDriver/src/tests/DriverTest.cpp -o CMakeFiles/DriverTest.dir/src/tests/DriverTest.s

CMakeFiles/DriverTest.dir/src/tests/DriverTest.o.requires:
.PHONY : CMakeFiles/DriverTest.dir/src/tests/DriverTest.o.requires

CMakeFiles/DriverTest.dir/src/tests/DriverTest.o.provides: CMakeFiles/DriverTest.dir/src/tests/DriverTest.o.requires
	$(MAKE) -f CMakeFiles/DriverTest.dir/build.make CMakeFiles/DriverTest.dir/src/tests/DriverTest.o.provides.build
.PHONY : CMakeFiles/DriverTest.dir/src/tests/DriverTest.o.provides

CMakeFiles/DriverTest.dir/src/tests/DriverTest.o.provides.build: CMakeFiles/DriverTest.dir/src/tests/DriverTest.o
.PHONY : CMakeFiles/DriverTest.dir/src/tests/DriverTest.o.provides.build

# Object files for target DriverTest
DriverTest_OBJECTS = \
"CMakeFiles/DriverTest.dir/src/tests/DriverTest.o"

# External object files for target DriverTest
DriverTest_EXTERNAL_OBJECTS =

../bin/DriverTest: CMakeFiles/DriverTest.dir/src/tests/DriverTest.o
../bin/DriverTest: ../lib/libre2robotDriver.so
../bin/DriverTest: CMakeFiles/DriverTest.dir/build.make
../bin/DriverTest: CMakeFiles/DriverTest.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../bin/DriverTest"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/DriverTest.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/DriverTest.dir/build: ../bin/DriverTest
.PHONY : CMakeFiles/DriverTest.dir/build

CMakeFiles/DriverTest.dir/requires: CMakeFiles/DriverTest.dir/src/tests/DriverTest.o.requires
.PHONY : CMakeFiles/DriverTest.dir/requires

CMakeFiles/DriverTest.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/DriverTest.dir/cmake_clean.cmake
.PHONY : CMakeFiles/DriverTest.dir/clean

CMakeFiles/DriverTest.dir/depend:
	cd /home/dave/ros/re2robot/re2robotDriver/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/dave/ros/re2robot/re2robotDriver /home/dave/ros/re2robot/re2robotDriver /home/dave/ros/re2robot/re2robotDriver/build /home/dave/ros/re2robot/re2robotDriver/build /home/dave/ros/re2robot/re2robotDriver/build/CMakeFiles/DriverTest.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/DriverTest.dir/depend

