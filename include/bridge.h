/*
 * Copyright (C) 2013 MACSi Project
 * Author: Woody Rousseau
 * email:  woody.rousseau@ensta-paristech.fr
 * website: www.macsi.isir.upmc.fr
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
*/

 /**
 * \file bridge.h
 * \brief Main header
 * \author Woody Rousseau
 * \version 0.1
 * \date 23/05/13
 *
 * Contains most commonly used libraries, namespaces and defines
 */

#ifndef H_BRIDGE
#define H_BRIDGE

#include <string>
#include <cstring>
#include <iostream>
//#include <unistd.h>
#include <errno.h>
#include <cstdlib>
#include <vector>
#include <utility>
#include "ros/ros.h"
#include <yarp/os/BufferedPort.h>

/**
 * \def P_READ
 * Used to get the read file descriptor. Is 0.
 */
#define P_READ 0
/**
 * \def P_WRITE
 * Used to get the write file descriptor. Is 0.
 */
#define P_WRITE 1

 // Used Namespaces and using declarations
using namespace std;
using namespace yarp::os;
/** This is used to hold all fields going through the bridge (type and name) */
typedef vector<pair<string,string>> field;

#endif

