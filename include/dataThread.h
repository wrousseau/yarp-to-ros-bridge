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
 * \file dataThread.h
 * \brief DataThread class
 * \author Woody Rousseau
 * \version 0.1
 * \date 23/05/13
 *
 * The thread sends/receives data and headers through the pipe
 */

#ifndef DATA_THREAD_H
#define DATA_THREAD_H

#include "bridge.h"
#include "convertThread.h"
#include "bridgeHeader.h"

extern FILE* fromYarpPipe; /**< We get that global FILE* because it is here that we write in it ! */

/** \class DataThread
   * \brief Sending data and headers through the pipe
   *
   * This class does almost all the work ! It opens the device port, let us connect with it
   * It then gathers data before sending it through the pipe
   */
class DataThread : public ConvertThread
{

private:
  BufferedPort<Bottle> inPort; /**< The port connected to the device port to gather data */
  field yarpGroups; /**< The headers corresponding to the data we are about to send */
  string headers; /**< Headers are all converted to one big string in order to be sent through the pipe */


public:

  /**
     *  \brief Constructor
     *
     *  Construct a DataThread from the parameters gathered in ConvertModule
     *
     * \param _name : The name of the module
     * \param _yarpPort : The device port from which we'll be seeking data
     * \param _rosTopic : The ROS topic on which we want to publish the data
     * \param _yarpGroups : The headers corresponding to the data we are about to send
     * \param _rate : The rate of the thread which is needed by the parent constructor
     * \param _fd : Pointer to the file descriptors 
     */
  DataThread(string _name, string _yarpPort, string _rosTopic, field _yarpGroups, int _rate, int *_fd);

  /**
     * \brief Initializing the thread
     *
     * The thread executes this function when it starts and before "run". 
     * The return value of threadInit() is notified to the class and passed as a parameter to afterStart()
     *
     * \return If the function returns false the thread quits and never calls "run"
     */
  virtual bool threadInit();

  /**
     *  \brief Releasing the thread
     *
     *  Mostly closing the client port
     */
  virtual void threadRelease();

  /**
     *  \brief Thread running at rate period
     *
     *  Does all the work, which is sending headers and data through the pipe to the other middleware
     */
  virtual void run();

};






#endif