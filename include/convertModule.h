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

#include <memory>
#include <signal.h>
#include <sys/stat.h>
#include <time.h>
#include <sys/wait.h>


// Yarp Librairies
#include <yarp/os/RFModule.h>
#include <yarp/os/Property.h>

// Macsi Libraries
#include <macsi/modHelp/modHelp.h>

#include "convertThread.h"
#include "bridgeHeader.h"
#include "fRBridge.h"
#include "pythonWrapper.h"
#include "bridge.h"

using namespace macsi::modHelp;

 /** \class ConvertModule convertModule.h convertModule
   * \brief Main Bridge Module common to both directions
   *
   * The ConvertModule class, inheriting from the YARP RFModule class. 
   * It reads the configuration file which is specific to this bridge and can try to use the automatic mode
   * It launches the bridge in the way asked by the configuration file.
   */
class ConvertModule: public RFModule
{

private:
	string name; /**< Name of the module */
	string bridgeType; /**< Type of the bridge (yarp2ros or ros2yarp) */
	string yarpPort; /**< Port used for YARP, either for input or output */
    string rosTopic; /**< Topic used for ROS, either for input or output */
    int dataType; /**< Data is not sent in the same way if they are basic types or if we want to send something else (an image) */
	int numberOfGroups; /**< How many data members are to be sent/received */
	field groups; /**< Map containing all the data members headers (their name and their type) */
	int rate; /**< Rate which will be used by the thread */

    unique_ptr<ConvertThread> thread; /**< A pointer to a ConvertThread. Only used for YARP->ROS */
    unique_ptr<FRBridge> fRBridge; /**< A pointer to a FRDataBridge. Only used for ROS->YARP */
    int *fd; /**< The file descriptors for read and write in the pipe. */
    bool isGui; /**< Is the program running in gui mode? */
    int automatic; /**< Are we trying to run the program in automatic mode ? */
    int count; /**< Incrementing variable to print that the bridge is alive each minute */
       
public:

    /**
     *  \brief Constructor
     *  \param fd the pointer to both file descriptors
     * \param gui true if the program is running in gui mode
     */  
    ConvertModule(int *fd, bool gui);

    /**
     *  \brief Configures the module, pass a ResourceFinder object to the module
     *  \param rf : a previously initialized ResourceFinder
     *  \return true/false upon success/failure
     *
     * This also launches the generation of the ROS .msg file, compiles the ROS modules and sends a signal to the children processes
     */    
    bool configure(ResourceFinder &rf);

    /**
     *  \brief Terminates the module
     *  \return true/false upon success/failure
     *
     * Closes the ports, frees memory and terminates the module
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

    /**
     *  \brief Figures out if there wasn't a mistake in the .ini file
     *  \return true/false upon success/failure
     *
     * Can be upgraded to do more verification.
     * It here just verifies that you're not trying to get image data as "classic" data
     */  
    bool checkPort(string devicePort, int dataType);

    /**
     *  \brief Compiles both ROS nodes using a bash script
     *  \param lastActualEdit a string giving the lastest date/time of edit of the .ini file
     *  \param dataType 0 if the bridge is dealing with data, 1 if it is dealing with images
     * 
     * Uses a std::thread to run the script
     */  
    void compileRosNodes(string lastActualEdit, int dataType);

    /**
     *  \brief Checks if the .ini file was modified since last execution
     *  \param rf The ResourceFinder to find the file path
     *  \param lastActualEdit will contain the latest edit of the .ini file
     *  \return True if the .ini file was edited
     *
     * If was not modified, the ROS module will not be recompiled
     */  
    bool checkUpdate(yarp::os::ResourceFinder &rf, string &lastActualEdit);

    /**
     *  \brief Tries to run in automatic mode
     *  \return True if the data on the port is managable in automatic mode
     *
     * For now, automatic mode works mainly with vectors of data from YARP->ROS. It should work for all cases from ROS->YARP
     */ 
    bool tryAutomaticMode();

    /**
     *  \brief YARP->ROS : Studies the bottle from the port to figure out what's in it
     *  \param b The YARP bottle from the port (see documentation)
     *  \param root The default name for the variable (here "v" will be followed by a number)
     *  \param size A reference to the number of elements from the port
     *  \param str A reference to a string which contains all types and names of data from the port
     *
     * For now, automatic mode works mainly with vectors of data, it will be made better over time
     */ 
    void getTypeString(Bottle& b, string root, int &size, string &str);

    /**
     *  \brief YARP->ROS : Getting the string which contains all types and names of data from the port
     *  \param type The type of the currently considered piece of data
     *  \param name The name of the currently considered piece of data
     *  \param size The number of elements from the currently considered data vector (1 if it's a scalar)
     *  \return The string which contians all types and names from the current data vector
     */ 
    string addPart(string type, string name, int code);
};

#endif

