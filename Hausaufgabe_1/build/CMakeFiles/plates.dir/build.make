# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/pan/code/Hausaufgabe

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pan/code/Hausaufgabe/build

# Include any dependencies generated for this target.
include CMakeFiles/plates.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/plates.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/plates.dir/flags.make

CMakeFiles/plates.dir/src/plates.cpp.o: CMakeFiles/plates.dir/flags.make
CMakeFiles/plates.dir/src/plates.cpp.o: ../src/plates.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pan/code/Hausaufgabe/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/plates.dir/src/plates.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/plates.dir/src/plates.cpp.o -c /home/pan/code/Hausaufgabe/src/plates.cpp

CMakeFiles/plates.dir/src/plates.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/plates.dir/src/plates.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pan/code/Hausaufgabe/src/plates.cpp > CMakeFiles/plates.dir/src/plates.cpp.i

CMakeFiles/plates.dir/src/plates.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/plates.dir/src/plates.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pan/code/Hausaufgabe/src/plates.cpp -o CMakeFiles/plates.dir/src/plates.cpp.s

# Object files for target plates
plates_OBJECTS = \
"CMakeFiles/plates.dir/src/plates.cpp.o"

# External object files for target plates
plates_EXTERNAL_OBJECTS =

plates: CMakeFiles/plates.dir/src/plates.cpp.o
plates: CMakeFiles/plates.dir/build.make
plates: /usr/local/lib/libopencv_dnn.so.3.4.15
plates: /usr/local/lib/libopencv_highgui.so.3.4.15
plates: /usr/local/lib/libopencv_ml.so.3.4.15
plates: /usr/local/lib/libopencv_objdetect.so.3.4.15
plates: /usr/local/lib/libopencv_shape.so.3.4.15
plates: /usr/local/lib/libopencv_stitching.so.3.4.15
plates: /usr/local/lib/libopencv_superres.so.3.4.15
plates: /usr/local/lib/libopencv_videostab.so.3.4.15
plates: /usr/local/lib/libopencv_viz.so.3.4.15
plates: /usr/local/lib/libopencv_calib3d.so.3.4.15
plates: /usr/local/lib/libopencv_features2d.so.3.4.15
plates: /usr/local/lib/libopencv_flann.so.3.4.15
plates: /usr/local/lib/libopencv_photo.so.3.4.15
plates: /usr/local/lib/libopencv_video.so.3.4.15
plates: /usr/local/lib/libopencv_videoio.so.3.4.15
plates: /usr/local/lib/libopencv_imgcodecs.so.3.4.15
plates: /usr/local/lib/libopencv_imgproc.so.3.4.15
plates: /usr/local/lib/libopencv_core.so.3.4.15
plates: CMakeFiles/plates.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pan/code/Hausaufgabe/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable plates"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/plates.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/plates.dir/build: plates

.PHONY : CMakeFiles/plates.dir/build

CMakeFiles/plates.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/plates.dir/cmake_clean.cmake
.PHONY : CMakeFiles/plates.dir/clean

CMakeFiles/plates.dir/depend:
	cd /home/pan/code/Hausaufgabe/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pan/code/Hausaufgabe /home/pan/code/Hausaufgabe /home/pan/code/Hausaufgabe/build /home/pan/code/Hausaufgabe/build /home/pan/code/Hausaufgabe/build/CMakeFiles/plates.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/plates.dir/depend
