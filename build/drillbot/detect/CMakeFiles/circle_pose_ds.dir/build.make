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
CMAKE_SOURCE_DIR = /home/luan/drill/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/luan/drill/build

# Include any dependencies generated for this target.
include drillbot/detect/CMakeFiles/circle_pose_ds.dir/depend.make

# Include the progress variables for this target.
include drillbot/detect/CMakeFiles/circle_pose_ds.dir/progress.make

# Include the compile flags for this target's objects.
include drillbot/detect/CMakeFiles/circle_pose_ds.dir/flags.make

drillbot/detect/CMakeFiles/circle_pose_ds.dir/src/circle_pose_ds.cpp.o: drillbot/detect/CMakeFiles/circle_pose_ds.dir/flags.make
drillbot/detect/CMakeFiles/circle_pose_ds.dir/src/circle_pose_ds.cpp.o: /home/luan/drill/src/drillbot/detect/src/circle_pose_ds.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/luan/drill/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object drillbot/detect/CMakeFiles/circle_pose_ds.dir/src/circle_pose_ds.cpp.o"
	cd /home/luan/drill/build/drillbot/detect && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/circle_pose_ds.dir/src/circle_pose_ds.cpp.o -c /home/luan/drill/src/drillbot/detect/src/circle_pose_ds.cpp

drillbot/detect/CMakeFiles/circle_pose_ds.dir/src/circle_pose_ds.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/circle_pose_ds.dir/src/circle_pose_ds.cpp.i"
	cd /home/luan/drill/build/drillbot/detect && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/luan/drill/src/drillbot/detect/src/circle_pose_ds.cpp > CMakeFiles/circle_pose_ds.dir/src/circle_pose_ds.cpp.i

drillbot/detect/CMakeFiles/circle_pose_ds.dir/src/circle_pose_ds.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/circle_pose_ds.dir/src/circle_pose_ds.cpp.s"
	cd /home/luan/drill/build/drillbot/detect && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/luan/drill/src/drillbot/detect/src/circle_pose_ds.cpp -o CMakeFiles/circle_pose_ds.dir/src/circle_pose_ds.cpp.s

drillbot/detect/CMakeFiles/circle_pose_ds.dir/src/circle_pose_ds.cpp.o.requires:
.PHONY : drillbot/detect/CMakeFiles/circle_pose_ds.dir/src/circle_pose_ds.cpp.o.requires

drillbot/detect/CMakeFiles/circle_pose_ds.dir/src/circle_pose_ds.cpp.o.provides: drillbot/detect/CMakeFiles/circle_pose_ds.dir/src/circle_pose_ds.cpp.o.requires
	$(MAKE) -f drillbot/detect/CMakeFiles/circle_pose_ds.dir/build.make drillbot/detect/CMakeFiles/circle_pose_ds.dir/src/circle_pose_ds.cpp.o.provides.build
.PHONY : drillbot/detect/CMakeFiles/circle_pose_ds.dir/src/circle_pose_ds.cpp.o.provides

drillbot/detect/CMakeFiles/circle_pose_ds.dir/src/circle_pose_ds.cpp.o.provides.build: drillbot/detect/CMakeFiles/circle_pose_ds.dir/src/circle_pose_ds.cpp.o

# Object files for target circle_pose_ds
circle_pose_ds_OBJECTS = \
"CMakeFiles/circle_pose_ds.dir/src/circle_pose_ds.cpp.o"

# External object files for target circle_pose_ds
circle_pose_ds_EXTERNAL_OBJECTS =

/home/luan/drill/devel/lib/detect/circle_pose_ds: drillbot/detect/CMakeFiles/circle_pose_ds.dir/src/circle_pose_ds.cpp.o
/home/luan/drill/devel/lib/detect/circle_pose_ds: drillbot/detect/CMakeFiles/circle_pose_ds.dir/build.make
/home/luan/drill/devel/lib/detect/circle_pose_ds: /opt/ros/indigo/lib/libcv_bridge.so
/home/luan/drill/devel/lib/detect/circle_pose_ds: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.2.4.8
/home/luan/drill/devel/lib/detect/circle_pose_ds: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
/home/luan/drill/devel/lib/detect/circle_pose_ds: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.2.4.8
/home/luan/drill/devel/lib/detect/circle_pose_ds: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.2.4.8
/home/luan/drill/devel/lib/detect/circle_pose_ds: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
/home/luan/drill/devel/lib/detect/circle_pose_ds: /usr/lib/x86_64-linux-gnu/libopencv_ocl.so.2.4.8
/home/luan/drill/devel/lib/detect/circle_pose_ds: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
/home/luan/drill/devel/lib/detect/circle_pose_ds: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
/home/luan/drill/devel/lib/detect/circle_pose_ds: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
/home/luan/drill/devel/lib/detect/circle_pose_ds: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
/home/luan/drill/devel/lib/detect/circle_pose_ds: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
/home/luan/drill/devel/lib/detect/circle_pose_ds: /usr/lib/x86_64-linux-gnu/libopencv_gpu.so.2.4.8
/home/luan/drill/devel/lib/detect/circle_pose_ds: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
/home/luan/drill/devel/lib/detect/circle_pose_ds: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
/home/luan/drill/devel/lib/detect/circle_pose_ds: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
/home/luan/drill/devel/lib/detect/circle_pose_ds: /usr/lib/x86_64-linux-gnu/libopencv_contrib.so.2.4.8
/home/luan/drill/devel/lib/detect/circle_pose_ds: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
/home/luan/drill/devel/lib/detect/circle_pose_ds: /opt/ros/indigo/lib/libimage_transport.so
/home/luan/drill/devel/lib/detect/circle_pose_ds: /opt/ros/indigo/lib/libmessage_filters.so
/home/luan/drill/devel/lib/detect/circle_pose_ds: /usr/lib/x86_64-linux-gnu/libtinyxml.so
/home/luan/drill/devel/lib/detect/circle_pose_ds: /opt/ros/indigo/lib/libclass_loader.so
/home/luan/drill/devel/lib/detect/circle_pose_ds: /usr/lib/libPocoFoundation.so
/home/luan/drill/devel/lib/detect/circle_pose_ds: /usr/lib/x86_64-linux-gnu/libdl.so
/home/luan/drill/devel/lib/detect/circle_pose_ds: /opt/ros/indigo/lib/libroslib.so
/home/luan/drill/devel/lib/detect/circle_pose_ds: /opt/ros/indigo/lib/libroscpp.so
/home/luan/drill/devel/lib/detect/circle_pose_ds: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/luan/drill/devel/lib/detect/circle_pose_ds: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/luan/drill/devel/lib/detect/circle_pose_ds: /opt/ros/indigo/lib/librosconsole.so
/home/luan/drill/devel/lib/detect/circle_pose_ds: /opt/ros/indigo/lib/librosconsole_log4cxx.so
/home/luan/drill/devel/lib/detect/circle_pose_ds: /opt/ros/indigo/lib/librosconsole_backend_interface.so
/home/luan/drill/devel/lib/detect/circle_pose_ds: /usr/lib/liblog4cxx.so
/home/luan/drill/devel/lib/detect/circle_pose_ds: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/luan/drill/devel/lib/detect/circle_pose_ds: /opt/ros/indigo/lib/libxmlrpcpp.so
/home/luan/drill/devel/lib/detect/circle_pose_ds: /opt/ros/indigo/lib/libroscpp_serialization.so
/home/luan/drill/devel/lib/detect/circle_pose_ds: /opt/ros/indigo/lib/librostime.so
/home/luan/drill/devel/lib/detect/circle_pose_ds: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/luan/drill/devel/lib/detect/circle_pose_ds: /opt/ros/indigo/lib/libcpp_common.so
/home/luan/drill/devel/lib/detect/circle_pose_ds: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/luan/drill/devel/lib/detect/circle_pose_ds: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/luan/drill/devel/lib/detect/circle_pose_ds: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/luan/drill/devel/lib/detect/circle_pose_ds: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/luan/drill/devel/lib/detect/circle_pose_ds: drillbot/detect/CMakeFiles/circle_pose_ds.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable /home/luan/drill/devel/lib/detect/circle_pose_ds"
	cd /home/luan/drill/build/drillbot/detect && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/circle_pose_ds.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
drillbot/detect/CMakeFiles/circle_pose_ds.dir/build: /home/luan/drill/devel/lib/detect/circle_pose_ds
.PHONY : drillbot/detect/CMakeFiles/circle_pose_ds.dir/build

drillbot/detect/CMakeFiles/circle_pose_ds.dir/requires: drillbot/detect/CMakeFiles/circle_pose_ds.dir/src/circle_pose_ds.cpp.o.requires
.PHONY : drillbot/detect/CMakeFiles/circle_pose_ds.dir/requires

drillbot/detect/CMakeFiles/circle_pose_ds.dir/clean:
	cd /home/luan/drill/build/drillbot/detect && $(CMAKE_COMMAND) -P CMakeFiles/circle_pose_ds.dir/cmake_clean.cmake
.PHONY : drillbot/detect/CMakeFiles/circle_pose_ds.dir/clean

drillbot/detect/CMakeFiles/circle_pose_ds.dir/depend:
	cd /home/luan/drill/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/luan/drill/src /home/luan/drill/src/drillbot/detect /home/luan/drill/build /home/luan/drill/build/drillbot/detect /home/luan/drill/build/drillbot/detect/CMakeFiles/circle_pose_ds.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : drillbot/detect/CMakeFiles/circle_pose_ds.dir/depend

