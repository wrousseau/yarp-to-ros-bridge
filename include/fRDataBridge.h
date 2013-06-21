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
 * \file fRDataBridge.h
 * \brief Sending data from a ROS topic to a YARP port
 * \author Woody Rousseau
 * \version 0.1
 * \date 05/06/13
 *
 */

#ifndef F_R_DATA_BRIDGE_H
#define F_R_DATA_BRIDGE_H

#include "pythonWrapper.h"
#include "fRBridge.h"

#include <iterator>
#include <algorithm>

#define STRING_T 0
#define DOUBLE_T 1
#define INT_T 2
#define BOOL_T 3

/** \class FRDataBridge
   * \brief Child class to deal with classic data from a ROS topic to a YARP port
   *
   * In this case, a Python wrapper must be used to write on the pipe because the message must be handled as a string in a generic way
   */
class FRDataBridge : public FRBridge
{

private:
    field rosGroups; /**< Vector of all the fields going through the bridge (type and name) */

public:
    /**
     *  \brief Constructor
     *
     * \param _fd : Pointer to the file descriptors
     * \param _topicName : Name of the ROS topic (used as input)
     * \param _yarpPort : Name of the YARP port (used as output)
     * \param _rosGroups : Vector of all the fields going through the bridge (type and name)
     */
    FRDataBridge(int* _fd, string _topicName, string _yarpPort, field _rosGroups);

    /**
     *  \brief Virtual method which handles writing the data on the pipe
     *
     * \return True/False upon success/Failure but won't normally return by itself
     *
     * Launches a Python scripts which grabs at a certain rate a message from the ROS topic, creates a string out of it, and sends it through the pipe
     */
    virtual bool writeData();
    
    /**
     *  \brief Virtual method which handles reading the data on the pipe
     *
     * \return True/False upon success/Failure but won't normally return by itself
     *
     * Will insert data in a YARP bottle and write on a BufferedPort
     * Depending on the type of each field, it will call the appropriate "add" function to insert them in the YARP bottle
     */
    virtual bool readRosData();

    /**
     * \brief String to Bool conversion
     * \param str can either be "true","false","0" or "1"
     * \return True or false depending on the input
     */
    bool boolStringToInt(string str);

    /**
     * \brief Get the YARP category (somewhat a YARP "type") to call the appropriate method to insert the field in the YARP bottle
     * \param type is the ROS type (automatic mode) or the type given in the .ini file (manual)
     * \return An int representing the category (string, double, int, bool)
     *
     * The bridge will never output the newest YARP types (List and Dict) but it's not a big deal
     */
    int getYarpTypeInt(string type);

    /**
     * \brief Algorithms that processes the big string coming out of the pipe to get data
     * \param receivedData the big string coming out of the pipe
     * \return A vector which contains each values accompanied by 1 if it's just one data, or an unsigned number if it is a vector
     *
     * Processing basically uses istream iterators but it also has some tricks to handle strings which contain whitespaces as well as dynamic size vectors
     */
    vector<pair<string,unsigned>> processIncomingData(const string &receivedData);
};

#endif
