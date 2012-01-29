cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
include(FindPkgConfig)
set(ROS_BUILD_TYPE Release)
rosbuild_init()

# Build tesseract
execute_process(COMMAND cmake -E chdir ${PROJECT_SOURCE_DIR} make -f Makefile.tesseract
                RESULT_VARIABLE _make_failed)
if(_make_failed)
  message(FATAL_ERROR "Build of tesseract failed")
endif(_make_failed)

set_directory_properties(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES tesseract-svn)


