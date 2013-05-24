##
 # Copyright (C) 2013 MACSi Project
 # Author: Woody Rousseau
 # email:  woody.rousseau@ensta-paristech.fr
 # website: www.macsi.isir.upmc.fr
 # Permission is granted to copy, distribute, and/or modify this program
 # under the terms of the GNU General Public License, version 2 or any
 # later version published by the Free Software Foundation.
 #
 # This program is distributed in the hope that it will be useful, but
 # WITHOUT ANY WARRANTY; without even the implied warranty of
 # MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 # Public License for more details
##

cmake_minimum_required(VERSION 2.6.4)

set(KEYWORD "yarptoros")
set(CMAKE_CXX_FLAGS "-std=c++0x")

# Start a project.
project(${KEYWORD})

# Find YARP.  Point the YARP_DIR environment variable at your build.
find_package(YARP REQUIRED)
find_package(OpenCV REQUIRED)

# Search for source code.
file(GLOB folder_source src/*.cpp src/*.cc src/*.c)
file(GLOB folder_header include/*.h)
source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

# Automatically add include directories if needed.
foreach(header_file ${folder_header})
  get_filename_component(p ${header_file} PATH)
  include_directories(${p})
endforeach(header_file ${folder_header})

# Inclue any directories needed for YARP
include_directories(${YARP_INCLUDE_DIRS}
		    ${modHelp_INCLUDE_DIRS}
		    ${objects_INCLUDE_DIRS})

# Set up our main executable.
if (folder_source)
  add_executable(${KEYWORD} ${folder_source} ${folder_header})
  target_link_libraries(${KEYWORD} ${YARP_LIBRARIES}
				   				   ${OpenCV_LIBS}
				   				   modHelp
				   				   objects)
else (folder_source)
  message(FATAL_ERROR "No source code files found. Please add something")
endif (folder_source)
