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

#include <yarp/os/all.h>
#include "dataToRos.h"

/** \struct BridgeHeader bridgeHeader.h bridgeHeader
   * \brief Agreggate class for headers sent/received
   *
   *  The headers are just the name and type of each data member sent/received 
   */
struct BridgeHeader
{
    string name; /**< Name of the data member to be sent/received */
    string type; /**< Type of the data member to be sent/received */
};

/** \fn void readBridgeHeaderVector(Bottle &rf, string name, vector<BridgeHeader> &v, int size)
   * \brief Interpreting the .ini file to fill a vector of headers
   * 
   * \param rf Bottle passed as a reference, group currently considered in the .ini file
   * \param name Either yarpgroups or rosgroups, depending on which vector we want to fill
   * \param v Vector passed as a reference. We want to fill it with the headers (just the name for now)
   * \param size Number of headers for that vector
   * 
   * This is based on the modHelp functions from a MACSI library  
   */
void readBridgeHeaderVector(Bottle &rf, string name, vector<BridgeHeader> &v, int size);

#endif