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
include drillbot/depthsense_cam/CMakeFiles/depthsense_cam_node.dir/depend.make

# Include the progress variables for this target.
include drillbot/depthsense_cam/CMakeFiles/depthsense_cam_node.dir/progress.make

# Include the compile flags for this target's objects.
include drillbot/depthsense_cam/CMakeFiles/depthsense_cam_node.dir/flags.make

drillbot/depthsense_cam/CMakeFiles/depthsense_cam_node.dir/src/depthsense_cam_node.cpp.o: drillbot/depthsense_cam/CMakeFiles/depthsense_cam_node.dir/flags.make
drillbot/depthsense_cam/CMakeFiles/depthsense_cam_node.dir/src/depthsense_cam_node.cpp.o: /home/luan/drill/src/drillbot/depthsense_cam/src/depthsense_cam_node.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/luan/drill/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object drillbot/depthsense_cam/CMakeFiles/depthsense_cam_node.dir/src/depthsense_cam_node.cpp.o"
	cd /home/luan/drill/build/drillbot/depthsense_cam && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/depthsense_cam_node.dir/src/depthsense_cam_node.cpp.o -c /home/luan/drill/src/drillbot/depthsense_cam/src/depthsense_cam_node.cpp

drillbot/depthsense_cam/CMakeFiles/depthsense_cam_node.dir/src/depthsense_cam_node.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/depthsense_cam_node.dir/src/depthsense_cam_node.cpp.i"
	cd /home/luan/drill/build/drillbot/depthsense_cam && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/luan/drill/src/drillbot/depthsense_cam/src/depthsense_cam_node.cpp > CMakeFiles/depthsense_cam_node.dir/src/depthsense_cam_node.cpp.i

drillbot/depthsense_cam/CMakeFiles/depthsense_cam_node.dir/src/depthsense_cam_node.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/depthsense_cam_node.dir/src/depthsense_cam_node.cpp.s"
	cd /home/luan/drill/build/drillbot/depthsense_cam && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/luan/drill/src/drillbot/depthsense_cam/src/depthsense_cam_node.cpp -o CMakeFiles/depthsense_cam_node.dir/src/depthsense_cam_node.cpp.s

drillbot/depthsense_cam/CMakeFiles/depthsense_cam_node.dir/src/depthsense_cam_node.cpp.o.requires:
.PHONY : drillbot/depthsense_cam/CMakeFiles/depthsense_cam_node.dir/src/depthsense_cam_node.cpp.o.requires

drillbot/depthsense_cam/CMakeFiles/depthsense_cam_node.dir/src/depthsense_cam_node.cpp.o.provides: drillbot/depthsense_cam/CMakeFiles/depthsense_cam_node.dir/src/depthsense_cam_node.cpp.o.requires
	$(MAKE) -f drillbot/depthsense_cam/CMakeFiles/depthsense_cam_node.dir/build.make drillbot/depthsense_cam/CMakeFiles/depthsense_cam_node.dir/src/depthsense_cam_node.cpp.o.provides.build
.PHONY : drillbot/depthsense_cam/CMakeFiles/depthsense_cam_node.dir/src/depthsense_cam_node.cpp.o.provides

drillbot/depthsense_cam/CMakeFiles/depthsense_cam_node.dir/src/depthsense_cam_node.cpp.o.provides.build: drillbot/depthsense_cam/CMakeFiles/depthsense_cam_node.dir/src/depthsense_cam_node.cpp.o

drillbot/depthsense_cam/CMakeFiles/depthsense_cam_node.dir/src/driver/src/depthsense_cam_driver.cpp.o: drillbot/depthsense_cam/CMakeFiles/depthsense_cam_node.dir/flags.make
drillbot/depthsense_cam/CMakeFiles/depthsense_cam_node.dir/src/driver/src/depthsense_cam_driver.cpp.o: /home/luan/drill/src/drillbot/depthsense_cam/src/driver/src/depthsense_cam_driver.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/luan/drill/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object drillbot/depthsense_cam/CMakeFiles/depthsense_cam_node.dir/src/driver/src/depthsense_cam_driver.cpp.o"
	cd /home/luan/drill/build/drillbot/depthsense_cam && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/depthsense_cam_node.dir/src/driver/src/depthsense_cam_driver.cpp.o -c /home/luan/drill/src/drillbot/depthsense_cam/src/driver/src/depthsense_cam_driver.cpp

drillbot/depthsense_cam/CMakeFiles/depthsense_cam_node.dir/src/driver/src/depthsense_cam_driver.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/depthsense_cam_node.dir/src/driver/src/depthsense_cam_driver.cpp.i"
	cd /home/luan/drill/build/drillbot/depthsense_cam && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/luan/drill/src/drillbot/depthsense_cam/src/driver/src/depthsense_cam_driver.cpp > CMakeFiles/depthsense_cam_node.dir/src/driver/src/depthsense_cam_driver.cpp.i

drillbot/depthsense_cam/CMakeFiles/depthsense_cam_node.dir/src/driver/src/depthsense_cam_driver.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/depthsense_cam_node.dir/src/driver/src/depthsense_cam_driver.cpp.s"
	cd /home/luan/drill/build/drillbot/depthsense_cam && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/luan/drill/src/drillbot/depthsense_cam/src/driver/src/depthsense_cam_driver.cpp -o CMakeFiles/depthsense_cam_node.dir/src/driver/src/depthsense_cam_driver.cpp.s

drillbot/depthsense_cam/CMakeFiles/depthsense_cam_node.dir/src/driver/src/depthsense_cam_driver.cpp.o.requires:
.PHONY : drillbot/depthsense_cam/CMakeFiles/depthsense_cam_node.dir/src/driver/src/depthsense_cam_driver.cpp.o.requires

drillbot/depthsense_cam/CMakeFiles/depthsense_cam_node.dir/src/driver/src/depthsense_cam_driver.cpp.o.provides: drillbot/depthsense_cam/CMakeFiles/depthsense_cam_node.dir/src/driver/src/depthsense_cam_driver.cpp.o.requires
	$(MAKE) -f drillbot/depthsense_cam/CMakeFiles/depthsense_cam_node.dir/build.make drillbot/depthsense_cam/CMakeFiles/depthsense_cam_node.dir/src/driver/src/depthsense_cam_driver.cpp.o.provides.build
.PHONY : drillbot/depthsense_cam/CMakeFiles/depthsense_cam_node.dir/src/driver/src/depthsense_cam_driver.cpp.o.provides

drillbot/depthsense_cam/CMakeFiles/depthsense_cam_node.dir/src/driver/src/depthsense_cam_driver.cpp.o.provides.build: drillbot/depthsense_cam/CMakeFiles/depthsense_cam_node.dir/src/driver/src/depthsense_cam_driver.cpp.o

# Object files for target depthsense_cam_node
depthsense_cam_node_OBJECTS = \
"CMakeFiles/depthsense_cam_node.dir/src/depthsense_cam_node.cpp.o" \
"CMakeFiles/depthsense_cam_node.dir/src/driver/src/depthsense_cam_driver.cpp.o"

# External object files for target depthsense_cam_node
depthsense_cam_node_EXTERNAL_OBJECTS =

/home/luan/drill/devel/lib/depthsense_cam/depthsense_cam_node: drillbot/depthsense_cam/CMakeFiles/depthsense_cam_node.dir/src/depthsense_cam_node.cpp.o
/home/luan/drill/devel/lib/depthsense_cam/depthsense_cam_node: drillbot/depthsense_cam/CMakeFiles/depthsense_cam_node.dir/src/driver/src/depthsense_cam_driver.cpp.o
/home/luan/drill/devel/lib/depthsense_cam/depthsense_cam_node: drillbot/depthsense_cam/CMakeFiles/depthsense_cam_node.dir/build.make
/home/luan/drill/devel/lib/depthsense_cam/depthsense_cam_node: /opt/ros/indigo/lib/libimage_transport.so
/home/luan/drill/devel/lib/depthsense_cam/depthsense_cam_node: /usr/lib/x86_64-linux-gnu/libtinyxml.so
/home/luan/drill/devel/lib/depthsense_cam/depthsense_cam_node: /opt/ros/indigo/lib/libclass_loader.so
/home/luan/drill/devel/lib/depthsense_cam/depthsense_cam_node: /usr/lib/libPocoFoundation.so
/home/luan/drill/devel/lib/depthsense_cam/depthsense_cam_node: /usr/lib/x86_64-linux-gnu/libdl.so
/home/luan/drill/devel/lib/depthsense_cam/depthsense_cam_node: /opt/ros/indigo/lib/libroslib.so
/home/luan/drill/devel/lib/depthsense_cam/depthsense_cam_node: /opt/ros/indigo/lib/libtf.so
/home/luan/drill/devel/lib/depthsense_cam/depthsense_cam_node: /opt/ros/indigo/lib/libtf2_ros.so
/home/luan/drill/devel/lib/depthsense_cam/depthsense_cam_node: /opt/ros/indigo/lib/libactionlib.so
/home/luan/drill/devel/lib/depthsense_cam/depthsense_cam_node: /opt/ros/indigo/lib/libmessage_filters.so
/home/luan/drill/devel/lib/depthsense_cam/depthsense_cam_node: /opt/ros/indigo/lib/libroscpp.so
/home/luan/drill/devel/lib/depthsense_cam/depthsense_cam_node: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/luan/drill/devel/lib/depthsense_cam/depthsense_cam_node: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/luan/drill/devel/lib/depthsense_cam/depthsense_cam_node: /opt/ros/indigo/lib/libxmlrpcpp.so
/home/luan/drill/devel/lib/depthsense_cam/depthsense_cam_node: /opt/ros/indigo/lib/libtf2.so
/home/luan/drill/devel/lib/depthsense_cam/depthsense_cam_node: /opt/ros/indigo/lib/libroscpp_serialization.so
/home/luan/drill/devel/lib/depthsense_cam/depthsense_cam_node: /opt/ros/indigo/lib/librosconsole.so
/home/luan/drill/devel/lib/depthsense_cam/depthsense_cam_node: /opt/ros/indigo/lib/librosconsole_log4cxx.so
/home/luan/drill/devel/lib/depthsense_cam/depthsense_cam_node: /opt/ros/indigo/lib/librosconsole_backend_interface.so
/home/luan/drill/devel/lib/depthsense_cam/depthsense_cam_node: /usr/lib/liblog4cxx.so
/home/luan/drill/devel/lib/depthsense_cam/depthsense_cam_node: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/luan/drill/devel/lib/depthsense_cam/depthsense_cam_node: /opt/ros/indigo/lib/librostime.so
/home/luan/drill/devel/lib/depthsense_cam/depthsense_cam_node: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/luan/drill/devel/lib/depthsense_cam/depthsense_cam_node: /opt/ros/indigo/lib/libcpp_common.so
/home/luan/drill/devel/lib/depthsense_cam/depthsense_cam_node: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/luan/drill/devel/lib/depthsense_cam/depthsense_cam_node: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/luan/drill/devel/lib/depthsense_cam/depthsense_cam_node: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/luan/drill/devel/lib/depthsense_cam/depthsense_cam_node: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/luan/drill/devel/lib/depthsense_cam/depthsense_cam_node: /opt/softkinetic/DepthSenseSDK/lib/libDepthSense.so
/home/luan/drill/devel/lib/depthsense_cam/depthsense_cam_node: drillbot/depthsense_cam/CMakeFiles/depthsense_cam_node.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable /home/luan/drill/devel/lib/depthsense_cam/depthsense_cam_node"
	cd /home/luan/drill/build/drillbot/depthsense_cam && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/depthsense_cam_node.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
drillbot/depthsense_cam/CMakeFiles/depthsense_cam_node.dir/build: /home/luan/drill/devel/lib/depthsense_cam/depthsense_cam_node
.PHONY : drillbot/depthsense_cam/CMakeFiles/depthsense_cam_node.dir/build

drillbot/depthsense_cam/CMakeFiles/depthsense_cam_node.dir/requires: drillbot/depthsense_cam/CMakeFiles/depthsense_cam_node.dir/src/depthsense_cam_node.cpp.o.requires
drillbot/depthsense_cam/CMakeFiles/depthsense_cam_node.dir/requires: drillbot/depthsense_cam/CMakeFiles/depthsense_cam_node.dir/src/driver/src/depthsense_cam_driver.cpp.o.requires
.PHONY : drillbot/depthsense_cam/CMakeFiles/depthsense_cam_node.dir/requires

drillbot/depthsense_cam/CMakeFiles/depthsense_cam_node.dir/clean:
	cd /home/luan/drill/build/drillbot/depthsense_cam && $(CMAKE_COMMAND) -P CMakeFiles/depthsense_cam_node.dir/cmake_clean.cmake
.PHONY : drillbot/depthsense_cam/CMakeFiles/depthsense_cam_node.dir/clean

drillbot/depthsense_cam/CMakeFiles/depthsense_cam_node.dir/depend:
	cd /home/luan/drill/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/luan/drill/src /home/luan/drill/src/drillbot/depthsense_cam /home/luan/drill/build /home/luan/drill/build/drillbot/depthsense_cam /home/luan/drill/build/drillbot/depthsense_cam/CMakeFiles/depthsense_cam_node.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : drillbot/depthsense_cam/CMakeFiles/depthsense_cam_node.dir/depend

