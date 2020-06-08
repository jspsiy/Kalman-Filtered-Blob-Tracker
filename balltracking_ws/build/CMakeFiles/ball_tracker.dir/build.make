# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.17

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

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/joshua/balltracking_ws

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/joshua/balltracking_ws/build

# Include any dependencies generated for this target.
include CMakeFiles/ball_tracker.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/ball_tracker.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/ball_tracker.dir/flags.make

CMakeFiles/ball_tracker.dir/src/balltrack.cpp.o: CMakeFiles/ball_tracker.dir/flags.make
CMakeFiles/ball_tracker.dir/src/balltrack.cpp.o: ../src/balltrack.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/joshua/balltracking_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/ball_tracker.dir/src/balltrack.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ball_tracker.dir/src/balltrack.cpp.o -c /home/joshua/balltracking_ws/src/balltrack.cpp

CMakeFiles/ball_tracker.dir/src/balltrack.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ball_tracker.dir/src/balltrack.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/joshua/balltracking_ws/src/balltrack.cpp > CMakeFiles/ball_tracker.dir/src/balltrack.cpp.i

CMakeFiles/ball_tracker.dir/src/balltrack.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ball_tracker.dir/src/balltrack.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/joshua/balltracking_ws/src/balltrack.cpp -o CMakeFiles/ball_tracker.dir/src/balltrack.cpp.s

CMakeFiles/ball_tracker.dir/src/KalmanObject.cpp.o: CMakeFiles/ball_tracker.dir/flags.make
CMakeFiles/ball_tracker.dir/src/KalmanObject.cpp.o: ../src/KalmanObject.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/joshua/balltracking_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/ball_tracker.dir/src/KalmanObject.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ball_tracker.dir/src/KalmanObject.cpp.o -c /home/joshua/balltracking_ws/src/KalmanObject.cpp

CMakeFiles/ball_tracker.dir/src/KalmanObject.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ball_tracker.dir/src/KalmanObject.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/joshua/balltracking_ws/src/KalmanObject.cpp > CMakeFiles/ball_tracker.dir/src/KalmanObject.cpp.i

CMakeFiles/ball_tracker.dir/src/KalmanObject.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ball_tracker.dir/src/KalmanObject.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/joshua/balltracking_ws/src/KalmanObject.cpp -o CMakeFiles/ball_tracker.dir/src/KalmanObject.cpp.s

# Object files for target ball_tracker
ball_tracker_OBJECTS = \
"CMakeFiles/ball_tracker.dir/src/balltrack.cpp.o" \
"CMakeFiles/ball_tracker.dir/src/KalmanObject.cpp.o"

# External object files for target ball_tracker
ball_tracker_EXTERNAL_OBJECTS =

ball_tracker: CMakeFiles/ball_tracker.dir/src/balltrack.cpp.o
ball_tracker: CMakeFiles/ball_tracker.dir/src/KalmanObject.cpp.o
ball_tracker: CMakeFiles/ball_tracker.dir/build.make
ball_tracker: /usr/local/lib/libopencv_gapi.so.4.1.0
ball_tracker: /usr/local/lib/libopencv_stitching.so.4.1.0
ball_tracker: /usr/local/lib/libopencv_aruco.so.4.1.0
ball_tracker: /usr/local/lib/libopencv_bgsegm.so.4.1.0
ball_tracker: /usr/local/lib/libopencv_bioinspired.so.4.1.0
ball_tracker: /usr/local/lib/libopencv_ccalib.so.4.1.0
ball_tracker: /usr/local/lib/libopencv_cvv.so.4.1.0
ball_tracker: /usr/local/lib/libopencv_dnn_objdetect.so.4.1.0
ball_tracker: /usr/local/lib/libopencv_dpm.so.4.1.0
ball_tracker: /usr/local/lib/libopencv_face.so.4.1.0
ball_tracker: /usr/local/lib/libopencv_freetype.so.4.1.0
ball_tracker: /usr/local/lib/libopencv_fuzzy.so.4.1.0
ball_tracker: /usr/local/lib/libopencv_hdf.so.4.1.0
ball_tracker: /usr/local/lib/libopencv_hfs.so.4.1.0
ball_tracker: /usr/local/lib/libopencv_img_hash.so.4.1.0
ball_tracker: /usr/local/lib/libopencv_line_descriptor.so.4.1.0
ball_tracker: /usr/local/lib/libopencv_quality.so.4.1.0
ball_tracker: /usr/local/lib/libopencv_reg.so.4.1.0
ball_tracker: /usr/local/lib/libopencv_rgbd.so.4.1.0
ball_tracker: /usr/local/lib/libopencv_saliency.so.4.1.0
ball_tracker: /usr/local/lib/libopencv_sfm.so.4.1.0
ball_tracker: /usr/local/lib/libopencv_stereo.so.4.1.0
ball_tracker: /usr/local/lib/libopencv_structured_light.so.4.1.0
ball_tracker: /usr/local/lib/libopencv_superres.so.4.1.0
ball_tracker: /usr/local/lib/libopencv_surface_matching.so.4.1.0
ball_tracker: /usr/local/lib/libopencv_tracking.so.4.1.0
ball_tracker: /usr/local/lib/libopencv_videostab.so.4.1.0
ball_tracker: /usr/local/lib/libopencv_viz.so.4.1.0
ball_tracker: /usr/local/lib/libopencv_xfeatures2d.so.4.1.0
ball_tracker: /usr/local/lib/libopencv_xobjdetect.so.4.1.0
ball_tracker: /usr/local/lib/libopencv_xphoto.so.4.1.0
ball_tracker: /usr/local/lib/libopencv_shape.so.4.1.0
ball_tracker: /usr/local/lib/libopencv_datasets.so.4.1.0
ball_tracker: /usr/local/lib/libopencv_plot.so.4.1.0
ball_tracker: /usr/local/lib/libopencv_text.so.4.1.0
ball_tracker: /usr/local/lib/libopencv_dnn.so.4.1.0
ball_tracker: /usr/local/lib/libopencv_ml.so.4.1.0
ball_tracker: /usr/local/lib/libopencv_phase_unwrapping.so.4.1.0
ball_tracker: /usr/local/lib/libopencv_optflow.so.4.1.0
ball_tracker: /usr/local/lib/libopencv_ximgproc.so.4.1.0
ball_tracker: /usr/local/lib/libopencv_video.so.4.1.0
ball_tracker: /usr/local/lib/libopencv_objdetect.so.4.1.0
ball_tracker: /usr/local/lib/libopencv_calib3d.so.4.1.0
ball_tracker: /usr/local/lib/libopencv_features2d.so.4.1.0
ball_tracker: /usr/local/lib/libopencv_flann.so.4.1.0
ball_tracker: /usr/local/lib/libopencv_highgui.so.4.1.0
ball_tracker: /usr/local/lib/libopencv_videoio.so.4.1.0
ball_tracker: /usr/local/lib/libopencv_imgcodecs.so.4.1.0
ball_tracker: /usr/local/lib/libopencv_photo.so.4.1.0
ball_tracker: /usr/local/lib/libopencv_imgproc.so.4.1.0
ball_tracker: /usr/local/lib/libopencv_core.so.4.1.0
ball_tracker: CMakeFiles/ball_tracker.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/joshua/balltracking_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable ball_tracker"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ball_tracker.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/ball_tracker.dir/build: ball_tracker

.PHONY : CMakeFiles/ball_tracker.dir/build

CMakeFiles/ball_tracker.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ball_tracker.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ball_tracker.dir/clean

CMakeFiles/ball_tracker.dir/depend:
	cd /home/joshua/balltracking_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/joshua/balltracking_ws /home/joshua/balltracking_ws /home/joshua/balltracking_ws/build /home/joshua/balltracking_ws/build /home/joshua/balltracking_ws/build/CMakeFiles/ball_tracker.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ball_tracker.dir/depend

