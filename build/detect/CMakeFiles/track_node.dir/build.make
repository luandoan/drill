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
include detect/CMakeFiles/track_node.dir/depend.make

# Include the progress variables for this target.
include detect/CMakeFiles/track_node.dir/progress.make

# Include the compile flags for this target's objects.
include detect/CMakeFiles/track_node.dir/flags.make

detect/CMakeFiles/track_node.dir/src/track_node.cpp.o: detect/CMakeFiles/track_node.dir/flags.make
detect/CMakeFiles/track_node.dir/src/track_node.cpp.o: /home/luan/drill/src/detect/src/track_node.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/luan/drill/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object detect/CMakeFiles/track_node.dir/src/track_node.cpp.o"
	cd /home/luan/drill/build/detect && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/track_node.dir/src/track_node.cpp.o -c /home/luan/drill/src/detect/src/track_node.cpp

detect/CMakeFiles/track_node.dir/src/track_node.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/track_node.dir/src/track_node.cpp.i"
	cd /home/luan/drill/build/detect && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/luan/drill/src/detect/src/track_node.cpp > CMakeFiles/track_node.dir/src/track_node.cpp.i

detect/CMakeFiles/track_node.dir/src/track_node.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/track_node.dir/src/track_node.cpp.s"
	cd /home/luan/drill/build/detect && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/luan/drill/src/detect/src/track_node.cpp -o CMakeFiles/track_node.dir/src/track_node.cpp.s

detect/CMakeFiles/track_node.dir/src/track_node.cpp.o.requires:
.PHONY : detect/CMakeFiles/track_node.dir/src/track_node.cpp.o.requires

detect/CMakeFiles/track_node.dir/src/track_node.cpp.o.provides: detect/CMakeFiles/track_node.dir/src/track_node.cpp.o.requires
	$(MAKE) -f detect/CMakeFiles/track_node.dir/build.make detect/CMakeFiles/track_node.dir/src/track_node.cpp.o.provides.build
.PHONY : detect/CMakeFiles/track_node.dir/src/track_node.cpp.o.provides

detect/CMakeFiles/track_node.dir/src/track_node.cpp.o.provides.build: detect/CMakeFiles/track_node.dir/src/track_node.cpp.o

# Object files for target track_node
track_node_OBJECTS = \
"CMakeFiles/track_node.dir/src/track_node.cpp.o"

# External object files for target track_node
track_node_EXTERNAL_OBJECTS =

/home/luan/drill/devel/lib/detect/track_node: detect/CMakeFiles/track_node.dir/src/track_node.cpp.o
/home/luan/drill/devel/lib/detect/track_node: detect/CMakeFiles/track_node.dir/build.make
/home/luan/drill/devel/lib/detect/track_node: /opt/ros/indigo/lib/libcv_bridge.so
/home/luan/drill/devel/lib/detect/track_node: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.2.4.8
/home/luan/drill/devel/lib/detect/track_node: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
/home/luan/drill/devel/lib/detect/track_node: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.2.4.8
/home/luan/drill/devel/lib/detect/track_node: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.2.4.8
/home/luan/drill/devel/lib/detect/track_node: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
/home/luan/drill/devel/lib/detect/track_node: /usr/lib/x86_64-linux-gnu/libopencv_ocl.so.2.4.8
/home/luan/drill/devel/lib/detect/track_node: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
/home/luan/drill/devel/lib/detect/track_node: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
/home/luan/drill/devel/lib/detect/track_node: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
/home/luan/drill/devel/lib/detect/track_node: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
/home/luan/drill/devel/lib/detect/track_node: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
/home/luan/drill/devel/lib/detect/track_node: /usr/lib/x86_64-linux-gnu/libopencv_gpu.so.2.4.8
/home/luan/drill/devel/lib/detect/track_node: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
/home/luan/drill/devel/lib/detect/track_node: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
/home/luan/drill/devel/lib/detect/track_node: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
/home/luan/drill/devel/lib/detect/track_node: /usr/lib/x86_64-linux-gnu/libopencv_contrib.so.2.4.8
/home/luan/drill/devel/lib/detect/track_node: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
/home/luan/drill/devel/lib/detect/track_node: /opt/ros/indigo/lib/libimage_transport.so
/home/luan/drill/devel/lib/detect/track_node: /opt/ros/indigo/lib/libmessage_filters.so
/home/luan/drill/devel/lib/detect/track_node: /usr/lib/x86_64-linux-gnu/libtinyxml.so
/home/luan/drill/devel/lib/detect/track_node: /opt/ros/indigo/lib/libclass_loader.so
/home/luan/drill/devel/lib/detect/track_node: /usr/lib/libPocoFoundation.so
/home/luan/drill/devel/lib/detect/track_node: /usr/lib/x86_64-linux-gnu/libdl.so
/home/luan/drill/devel/lib/detect/track_node: /opt/ros/indigo/lib/libroslib.so
/home/luan/drill/devel/lib/detect/track_node: /opt/ros/indigo/lib/libroscpp.so
/home/luan/drill/devel/lib/detect/track_node: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/luan/drill/devel/lib/detect/track_node: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/luan/drill/devel/lib/detect/track_node: /opt/ros/indigo/lib/librosconsole.so
/home/luan/drill/devel/lib/detect/track_node: /opt/ros/indigo/lib/librosconsole_log4cxx.so
/home/luan/drill/devel/lib/detect/track_node: /opt/ros/indigo/lib/librosconsole_backend_interface.so
/home/luan/drill/devel/lib/detect/track_node: /usr/lib/liblog4cxx.so
/home/luan/drill/devel/lib/detect/track_node: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/luan/drill/devel/lib/detect/track_node: /opt/ros/indigo/lib/libxmlrpcpp.so
/home/luan/drill/devel/lib/detect/track_node: /opt/ros/indigo/lib/libroscpp_serialization.so
/home/luan/drill/devel/lib/detect/track_node: /opt/ros/indigo/lib/librostime.so
/home/luan/drill/devel/lib/detect/track_node: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/luan/drill/devel/lib/detect/track_node: /opt/ros/indigo/lib/libcpp_common.so
/home/luan/drill/devel/lib/detect/track_node: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/luan/drill/devel/lib/detect/track_node: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/luan/drill/devel/lib/detect/track_node: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/luan/drill/devel/lib/detect/track_node: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/luan/drill/devel/lib/detect/track_node: detect/CMakeFiles/track_node.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable /home/luan/drill/devel/lib/detect/track_node"
	cd /home/luan/drill/build/detect && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/track_node.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
detect/CMakeFiles/track_node.dir/build: /home/luan/drill/devel/lib/detect/track_node
.PHONY : detect/CMakeFiles/track_node.dir/build

detect/CMakeFiles/track_node.dir/requires: detect/CMakeFiles/track_node.dir/src/track_node.cpp.o.requires
.PHONY : detect/CMakeFiles/track_node.dir/requires

detect/CMakeFiles/track_node.dir/clean:
	cd /home/luan/drill/build/detect && $(CMAKE_COMMAND) -P CMakeFiles/track_node.dir/cmake_clean.cmake
.PHONY : detect/CMakeFiles/track_node.dir/clean

detect/CMakeFiles/track_node.dir/depend:
	cd /home/luan/drill/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/luan/drill/src /home/luan/drill/src/detect /home/luan/drill/build /home/luan/drill/build/detect /home/luan/drill/build/detect/CMakeFiles/track_node.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : detect/CMakeFiles/track_node.dir/depend

