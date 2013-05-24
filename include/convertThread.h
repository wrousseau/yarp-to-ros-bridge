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
 * \file convertThread.h
 * \brief ConvertThread class
 * \author Woody Rousseau
 * \version 0.1
 * \date 23/05/13
 *
 * The thread sends/receives data and headers through the pipe
 */

#ifndef D_CONVERT_THREAD
#define D_CONVERT_THREAD

// Yarp Libraries
#include <yarp/os/RateThread.h>
#include <yarp/os/BufferedPort.h>

#include "dataToRos.h"

/** \class ConvertThread
   * \brief Sending data and headers through the pipe
   *
   * This class does almost all the work ! It opens the device port, let us connect with it
   * It then gathers data before sending it through the pipe
   */
class ConvertThread : public RateThread
{

protected:
  string name; /**< The name of the module */
  string robot; /**< The name of the robot */
  string devicePort; /**< The device port from which we'll be seeking data */

  bool firstData; /**< Is this the first data we're sending in a message ? */

public:

  /**
     *  \brief Constructor
     *
     *  Construct a ConvertThread from the parameters gathered in ConvertModule
     *
     * \param _name : The name of the module
     * \param _robot : The name of the robot
     * \param _devicePort : The device port from which we'll be seeking data
     * \param _yarpGroups : The headers corresponding to the data we are about to send
     * \param _rate : The rate of the thread which is needed by the parent constructor
     */
  ConvertThread(string _name, string _robot, string _devicePort, int _rate);
};

#endif