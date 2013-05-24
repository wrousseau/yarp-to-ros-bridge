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
 * \file convertModule.h
 * \brief ConvertModule class
 * \author Woody Rousseau
 * \version 0.1
 * \date 23/05/13
 */

#ifndef H_CONVERTMODULE
#define H_CONVERTMODULE

// Yarp Librairies
#include <yarp/os/RFModule.h>
#include <yarp/os/Property.h>

// Macsi Libraries
#include <macsi/modHelp/modHelp.h>

#include "dataToRos.h"
#include "convertThread.h"
#include "bridgeHeader.h"

using namespace macsi::modHelp;

 /** \class ConvertModule convertModule.h convertModule
   * \brief Agreggate class for headers sent/received
   *
   * The ConvertModule class, inheriting from the YARP RFModule class. 
   * It mostly reads the configuration file which is specific to this bridge.
   * It also allows rpc communication with the device
   */
class ConvertModule: public RFModule
{

private:
	// Parameters from .ini file
	string name; /**< Name of the module */
	string robot; /**< Name of the robot */
	string bridgeType; /**< Type of the bridge (yarp2ros or ros2yarp) */
	string devicePort; /**< Port used by the device (without the robot name) */
    int dataType; /**< Data is not sent in the same way if they are basic types or if we want to send something else (an image) */
	int numberOfYarpGroups; /**< How many YARP data members are to be sent/received */
	int numberOfRosGroups; /**< How many ROS data members are to be sent/received */
	vector<BridgeHeader> yarpGroups; /**< Vector containing all YARP data members headers (their name and their value) */
	vector<BridgeHeader> rosGroups; /**< Vector containing all ROS data members headers (their name and their value) */
	int rate; /**< Rate which will be used by the thread */

    ConvertThread *thread; /**< A pointer to a ConvertThread. It will eventually point to a derived class (image or data) */
    Port rpcPort; /**< Port used to send rpc commands to the device */
       
public:
    /**
     *  \brief Configures the module, pass a ResourceFinder object to the module
     *  \param rf : a previously initialized ResourceFinder
     *  \return true/false upon success/failure
     */    
    bool configure(ResourceFinder &rf);

    /**
     *  \brief Terminates the module
     *  \return true/false upon success/failure
     *
     * Closes the ports, frees the thread and terminates the module
     */  
    bool close();
    
    /**
     *  \brief You can override this to control the approximate periodicity at which updateModule() is called by runModule()
     *  \return the desired period between successive calls to updateModule()
	 *
	 *  By default, it returns 1.0. Time is here in seconds.
     */   
    virtual double getPeriod();

    /**
     *  \brief Main work of the module
     *  \return true
     *
     * The thread does all the work, so this does nothing.
     * However, because it is a virtual method, we implemented it.
     */  
    virtual bool updateModule();

};

#endif

