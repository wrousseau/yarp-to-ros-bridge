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
 * \file fRBridge.h
 * \brief Bridge from a ROS topic to a YARP port
 * \author Woody Rousseau
 * \version 0.1
 * \date 17/06/13
 *
 * This just launches both writing and reading threads and implements the pure virtual methods which will be called by these threads
 */

#ifndef F_R_BRIDGE_H
#define F_R_BRIDGE_H

#include <thread>

#include "bridge.h"

/** \class FRBridge
   * \brief Father class used for the bridge from a ROS topic to a YARP port
   *
   * The class only holds as attributes the file descriptors and the names of the port and the topic
   */
class FRBridge
{
protected:
    int *fd; /**< Pointer to the file descriptors */
    string topicName; /**< Name of the ROS topic (used as input) */
    string yarpPort; /**< Name of the YARP port (used as output) */

public:
      /**
     *  \brief Constructor
     *
     * \param _fd : Pointer to the file descriptors
     * \param _topicName : Name of the ROS topic (used as input)
     * \param _yarpPort : Name of the YARP port (used as output)
     */
    FRBridge(int* _fd, string _topicName, string _yarpPort);

    /**
     *  \brief Virtual method which handles writing the data on the pipe
     *
     * \return True/False upon success/Failure but won't normally return by itself
     */
    virtual bool writeData() = 0;

    /**
     *  \brief Virtual method which handles reading the data from the pipe
     *
     * \return True/False upon success/Failure but won't normally return by itself
     */
    virtual bool readRosData() = 0;

    /**
     *  \brief Launches both writing and reading threads
     *
     * \return True/False upon success/Failure
     */
    bool run();
};

#endif
