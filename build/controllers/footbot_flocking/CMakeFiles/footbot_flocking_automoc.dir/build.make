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
CMAKE_SOURCE_DIR = /home/cch-student/catkin_ws/src/argos3-examples

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/cch-student/catkin_ws/src/argos3-examples/build

# Utility rule file for footbot_flocking_automoc.

# Include the progress variables for this target.
include controllers/footbot_flocking/CMakeFiles/footbot_flocking_automoc.dir/progress.make

controllers/footbot_flocking/CMakeFiles/footbot_flocking_automoc:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/cch-student/catkin_ws/src/argos3-examples/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Automatic moc for target footbot_flocking"
	cd /home/cch-student/catkin_ws/src/argos3-examples/build/controllers/footbot_flocking && /usr/bin/cmake -E cmake_autogen /home/cch-student/catkin_ws/src/argos3-examples/build/controllers/footbot_flocking/CMakeFiles/footbot_flocking_automoc.dir/ Release

footbot_flocking_automoc: controllers/footbot_flocking/CMakeFiles/footbot_flocking_automoc
footbot_flocking_automoc: controllers/footbot_flocking/CMakeFiles/footbot_flocking_automoc.dir/build.make

.PHONY : footbot_flocking_automoc

# Rule to build all files generated by this target.
controllers/footbot_flocking/CMakeFiles/footbot_flocking_automoc.dir/build: footbot_flocking_automoc

.PHONY : controllers/footbot_flocking/CMakeFiles/footbot_flocking_automoc.dir/build

controllers/footbot_flocking/CMakeFiles/footbot_flocking_automoc.dir/clean:
	cd /home/cch-student/catkin_ws/src/argos3-examples/build/controllers/footbot_flocking && $(CMAKE_COMMAND) -P CMakeFiles/footbot_flocking_automoc.dir/cmake_clean.cmake
.PHONY : controllers/footbot_flocking/CMakeFiles/footbot_flocking_automoc.dir/clean

controllers/footbot_flocking/CMakeFiles/footbot_flocking_automoc.dir/depend:
	cd /home/cch-student/catkin_ws/src/argos3-examples/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cch-student/catkin_ws/src/argos3-examples /home/cch-student/catkin_ws/src/argos3-examples/controllers/footbot_flocking /home/cch-student/catkin_ws/src/argos3-examples/build /home/cch-student/catkin_ws/src/argos3-examples/build/controllers/footbot_flocking /home/cch-student/catkin_ws/src/argos3-examples/build/controllers/footbot_flocking/CMakeFiles/footbot_flocking_automoc.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : controllers/footbot_flocking/CMakeFiles/footbot_flocking_automoc.dir/depend

