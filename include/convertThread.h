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

#include <fstream>
#include <signal.h>


// Yarp Libraries
#include <yarp/os/RateThread.h>

#include "bridge.h"

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
  string yarpPort; /**< The YARP port from which we'll be seeking data */
  string rosTopic; /**< The name of the ROS topic we want to publish on */

  bool firstData; /**< Is this the first data we're sending in a message ? */
  int *fd; /**< The file descriptors for the pipe */
  const char *pathName;
  pid_t childPid;
  struct sigaction sigact;


public:

  /**
     *  \brief Constructor
     *
     *  Construct a ConvertThread from the parameters gathered in ConvertModule
     *
     * \param _name : The name of the module
     * \param _yarpPort : The YARP port from which we'll be seeking data
     * \param _rosTopic : The name of the ROS topic we want to publish on
     * \param _rate : The rate of the thread which is needed by the parent constructor
     * \param _fd : Pointer to the file descriptors for the pipe 
     */
  ConvertThread(string _name, string _yarpPort, string _rosTopic, int _rate, int *_fd);


  /**
  * \fn void openRosSender(int *fd)
  * \brief Creates a child process which is replaced by rosrun, launching the ros module whose goal is to publish data to a ROS topic
  */
  void openRosSender();

  /**
  * \fn void launchRosModule(int type)
  * \brief Launches the ros module to receive data from the pipe
  * \param pipeNumberStr the "read" file descriptor in a string to be passed as an argument
  */
  void launchRosModule();

};

  /**
  * \fn void catcher(int sig)
  * \brief Signal handling callback function
  * \param Will allow the writing side to write some more by setting "loopAgain" to true
  */
void catcher (int sig);


#endif