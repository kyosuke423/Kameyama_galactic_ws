# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.26

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

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /home/dars/.local/lib/python3.8/site-packages/cmake/data/bin/cmake

# The command to remove a file.
RM = /home/dars/.local/lib/python3.8/site-packages/cmake/data/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/dars/kameyama_galactic_ws/src/cuboid_analysis

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/dars/kameyama_galactic_ws/build/cuboid_analysis

# Include any dependencies generated for this target.
include CMakeFiles/ave_cov.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/ave_cov.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/ave_cov.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/ave_cov.dir/flags.make

CMakeFiles/ave_cov.dir/src/ave_cov.cpp.o: CMakeFiles/ave_cov.dir/flags.make
CMakeFiles/ave_cov.dir/src/ave_cov.cpp.o: /home/dars/kameyama_galactic_ws/src/cuboid_analysis/src/ave_cov.cpp
CMakeFiles/ave_cov.dir/src/ave_cov.cpp.o: CMakeFiles/ave_cov.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/dars/kameyama_galactic_ws/build/cuboid_analysis/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/ave_cov.dir/src/ave_cov.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/ave_cov.dir/src/ave_cov.cpp.o -MF CMakeFiles/ave_cov.dir/src/ave_cov.cpp.o.d -o CMakeFiles/ave_cov.dir/src/ave_cov.cpp.o -c /home/dars/kameyama_galactic_ws/src/cuboid_analysis/src/ave_cov.cpp

CMakeFiles/ave_cov.dir/src/ave_cov.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ave_cov.dir/src/ave_cov.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/dars/kameyama_galactic_ws/src/cuboid_analysis/src/ave_cov.cpp > CMakeFiles/ave_cov.dir/src/ave_cov.cpp.i

CMakeFiles/ave_cov.dir/src/ave_cov.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ave_cov.dir/src/ave_cov.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/dars/kameyama_galactic_ws/src/cuboid_analysis/src/ave_cov.cpp -o CMakeFiles/ave_cov.dir/src/ave_cov.cpp.s

# Object files for target ave_cov
ave_cov_OBJECTS = \
"CMakeFiles/ave_cov.dir/src/ave_cov.cpp.o"

# External object files for target ave_cov
ave_cov_EXTERNAL_OBJECTS =

ave_cov: CMakeFiles/ave_cov.dir/src/ave_cov.cpp.o
ave_cov: CMakeFiles/ave_cov.dir/build.make
ave_cov: /opt/ros/galactic/lib/librclcpp.so
ave_cov: /opt/ros/galactic/lib/libament_index_cpp.so
ave_cov: /opt/ros/galactic/lib/liblibstatistics_collector.so
ave_cov: /opt/ros/galactic/lib/liblibstatistics_collector_test_msgs__rosidl_typesupport_introspection_c.so
ave_cov: /opt/ros/galactic/lib/liblibstatistics_collector_test_msgs__rosidl_generator_c.so
ave_cov: /opt/ros/galactic/lib/liblibstatistics_collector_test_msgs__rosidl_typesupport_c.so
ave_cov: /opt/ros/galactic/lib/liblibstatistics_collector_test_msgs__rosidl_typesupport_introspection_cpp.so
ave_cov: /opt/ros/galactic/lib/liblibstatistics_collector_test_msgs__rosidl_typesupport_cpp.so
ave_cov: /opt/ros/galactic/lib/libstd_msgs__rosidl_typesupport_introspection_c.so
ave_cov: /opt/ros/galactic/lib/libstd_msgs__rosidl_generator_c.so
ave_cov: /opt/ros/galactic/lib/libstd_msgs__rosidl_typesupport_c.so
ave_cov: /opt/ros/galactic/lib/libstd_msgs__rosidl_typesupport_introspection_cpp.so
ave_cov: /opt/ros/galactic/lib/libstd_msgs__rosidl_typesupport_cpp.so
ave_cov: /opt/ros/galactic/lib/librcl.so
ave_cov: /opt/ros/galactic/lib/librcl_interfaces__rosidl_typesupport_introspection_c.so
ave_cov: /opt/ros/galactic/lib/librcl_interfaces__rosidl_generator_c.so
ave_cov: /opt/ros/galactic/lib/librcl_interfaces__rosidl_typesupport_c.so
ave_cov: /opt/ros/galactic/lib/librcl_interfaces__rosidl_typesupport_introspection_cpp.so
ave_cov: /opt/ros/galactic/lib/librcl_interfaces__rosidl_typesupport_cpp.so
ave_cov: /opt/ros/galactic/lib/librmw_implementation.so
ave_cov: /opt/ros/galactic/lib/librcl_logging_spdlog.so
ave_cov: /opt/ros/galactic/lib/librcl_logging_interface.so
ave_cov: /opt/ros/galactic/lib/librcl_yaml_param_parser.so
ave_cov: /opt/ros/galactic/lib/librmw.so
ave_cov: /opt/ros/galactic/lib/libyaml.so
ave_cov: /opt/ros/galactic/lib/librosgraph_msgs__rosidl_typesupport_introspection_c.so
ave_cov: /opt/ros/galactic/lib/librosgraph_msgs__rosidl_generator_c.so
ave_cov: /opt/ros/galactic/lib/librosgraph_msgs__rosidl_typesupport_c.so
ave_cov: /opt/ros/galactic/lib/librosgraph_msgs__rosidl_typesupport_introspection_cpp.so
ave_cov: /opt/ros/galactic/lib/librosgraph_msgs__rosidl_typesupport_cpp.so
ave_cov: /opt/ros/galactic/lib/libstatistics_msgs__rosidl_typesupport_introspection_c.so
ave_cov: /opt/ros/galactic/lib/libstatistics_msgs__rosidl_generator_c.so
ave_cov: /opt/ros/galactic/lib/libstatistics_msgs__rosidl_typesupport_c.so
ave_cov: /opt/ros/galactic/lib/libstatistics_msgs__rosidl_typesupport_introspection_cpp.so
ave_cov: /opt/ros/galactic/lib/libstatistics_msgs__rosidl_typesupport_cpp.so
ave_cov: /opt/ros/galactic/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_c.so
ave_cov: /opt/ros/galactic/lib/libbuiltin_interfaces__rosidl_generator_c.so
ave_cov: /opt/ros/galactic/lib/libbuiltin_interfaces__rosidl_typesupport_c.so
ave_cov: /opt/ros/galactic/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_cpp.so
ave_cov: /opt/ros/galactic/lib/librosidl_typesupport_introspection_cpp.so
ave_cov: /opt/ros/galactic/lib/librosidl_typesupport_introspection_c.so
ave_cov: /opt/ros/galactic/lib/libbuiltin_interfaces__rosidl_typesupport_cpp.so
ave_cov: /opt/ros/galactic/lib/librosidl_typesupport_cpp.so
ave_cov: /opt/ros/galactic/lib/librosidl_typesupport_c.so
ave_cov: /opt/ros/galactic/lib/librcpputils.so
ave_cov: /opt/ros/galactic/lib/librosidl_runtime_c.so
ave_cov: /opt/ros/galactic/lib/librcutils.so
ave_cov: /opt/ros/galactic/lib/libtracetools.so
ave_cov: CMakeFiles/ave_cov.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/dars/kameyama_galactic_ws/build/cuboid_analysis/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ave_cov"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ave_cov.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/ave_cov.dir/build: ave_cov
.PHONY : CMakeFiles/ave_cov.dir/build

CMakeFiles/ave_cov.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ave_cov.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ave_cov.dir/clean

CMakeFiles/ave_cov.dir/depend:
	cd /home/dars/kameyama_galactic_ws/build/cuboid_analysis && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/dars/kameyama_galactic_ws/src/cuboid_analysis /home/dars/kameyama_galactic_ws/src/cuboid_analysis /home/dars/kameyama_galactic_ws/build/cuboid_analysis /home/dars/kameyama_galactic_ws/build/cuboid_analysis /home/dars/kameyama_galactic_ws/build/cuboid_analysis/CMakeFiles/ave_cov.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ave_cov.dir/depend

