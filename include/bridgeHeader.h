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
 * \file bridgeHeader.h
 * \brief Representing headers sent/received and related functions 
 * \author Woody Rousseau
 * \version 0.1
 * \date 23/05/13
 *
 */

#ifndef BRIDGE_HEADER_H
#define BRIDGE_HEADER_H

#include <fstream>
#include <algorithm>
#include <sys/stat.h>
#include <yarp/os/all.h>
#include "bridge.h"


/** \fn void readBridgeHeaderVector(Bottle &rf, string name, vector<BridgeHeader> &v, int size)
   * \brief Interpreting the .ini file to fill a vector of headers
   * 
   * \param rf Bottle passed as a reference, group currently considered in the .ini file
   * \param name Either yarpgroups or rosgroups, depending on which vector we want to fill
   * \param groups passed as a reference. Elements will be pushed back, but only the name of the data for now
   * \param size Number of headers for that vector
   * 
   * This is based on the modHelp functions from a MACSI library  
   */
void readBridgeHeaderVector(Bottle &rf, string name, field &groups, int size);

/** \fn void generateRosMessage(vector<BridgeHeader> &v)
   * \brief Generates the ROS .msg file before compilation
   * \param groups passed as a reference to const in order to write the proper .msg file using an ofstream
   * 
   * This uses an ofstream to generate the Data.msg file in "ros_bridge/msg/"  
   */
void generateRosMessage(const field &groups);

/** \fn string convertToRos(string type)
   * \brief Converts the types from the .ini files into ROS understandable types
   * \param type The type written in the .ini file
   * 
   * If a type does not match a corresponding ROS type, the user is warned but the written type is kept. It might not work.  
   */
string convertToRos(string type);

#endif