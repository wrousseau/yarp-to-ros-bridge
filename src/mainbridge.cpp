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
 * \file mainbridge.cpp
 * \brief Main entry to the program
 * \author Woody Rousseau
 * \version 0.1
 * \date 23/05/13
 *
 * We here launch the ros programs by overlapping children processes created using fork()
 */

// Yarp Librairies
#include <yarp/os/Network.h>
#include <yarp/os/Property.h>

#include <sstream>
#include <signal.h>

#include "dataToRos.h"
#include "convertModule.h"

using namespace yarp::os;

FILE* yarpPipe; // In order to deal with file descriptors, we must use a C FILE*

/**
 * \fn void openRosSender(int *fd)
 * \brief Creates a child process which is replaced by rosrun, launching the ros module whose goal is to publish data to a ROS topic
 * \param fd Pointer to the two file descriptors, the "read" file descriptor will be handled by the ros module to get the data
 * \return Nothing if all went well, will exit(EXIT_FAILURE) if something went wrong
 */
void openRosSender(int *fd);

/**
 * \fn void openRosReceiver()
 * \brief Creates a child process which is replaced by rosrun, launching the ros module whose goal is to subscribe to the same ROS topic
 *
 * \return Nothing if all went well, will exit(EXIT_FAILURE) if something went wrong
 */
void openRosReceiver();

 /**
  * \fn void killFatherAndExit()
  * \brief Sends a SIGTGERM signal to the father and exits when something went wrong
  *
  */
void killFatherAndExit();

/**
 * \fn int main (int argc, char **argv)
 * \brief Main entry for the program, creates the pipe.
 *
 * \return EXIT_SUCCESS - The program has successfully ran
 */
int main(int argc, char **argv) 
{   
    // We initialize two file descriptors that form the pipe
    // fd[0] is the "Read" side of the pipe and fd[1] the "Write" side       
    int fd[2];
    if ( pipe(fd) == -1 ) 
    {
        cerr << "[yarp]::Pipe Error : " << strerror(errno) << endl;
        std::exit(EXIT_FAILURE);
    }
       
    // Forking and launching the ROS modules
    openRosSender(fd);
    openRosReceiver();        
    
    // Only the original process (father) has survived, he's the one which will be writing, so we don't need the "read" side    
    close(fd[P_READ]);
    // Let's open the pipe in write mode
    yarpPipe = fdopen(fd[P_WRITE], "w");
    if (yarpPipe == NULL)
    {
        cerr << "[yarp]::The pipe could not be opened (write mode) : " << strerror(errno) << endl;
        std::exit(EXIT_FAILURE);
    }
    
    // Setting up YARP
    Network yarp;
    if (!yarp.checkNetwork())
    {
        cerr << "[yarp]::The YARP server is not available!" << endl; 
        std::exit(EXIT_FAILURE);
    }
    
    // Let's load the parameters (.ini file)
    ResourceFinder rf;
    rf.setVerbose(true); 
    rf.setDefaultContext("ros/conf");
    rf.setDefaultConfigFile("bridge.ini");
    rf.configure("MACSI_ROOT",argc,argv);

    // We create the module which will be handling the conversion
    ConvertModule module; 
    module.configure(rf); // We pass the ResourceFinder object
    module.runModule();

    std::exit(EXIT_SUCCESS); // All went well !
}

void openRosSender(int *fd)
{
    string pipeNumberStr; // We need the file descriptor as a string
    ostringstream oss; // To handle the conversion from int to string
    switch (fork()) 
    {
        case -1 : // Something went wrong. Abort.
            cerr << "Fork instruction has failed (sender) : " << strerror(errno) << endl;
            std::exit(EXIT_FAILURE);
    
        case 0 : // Child process of the fork()
            close(fd[P_WRITE]); // We don't need the "Write" side of the pipe, we're just reading
    
            yarpPipe = fdopen(fd[P_READ], "r");
            if (yarpPipe == NULL) 
            {
                cerr << "File failed to open in read mode (pipe) : " << strerror(errno) << endl;
                killFatherAndExit();
            }

            oss << fd[P_READ]; // Getting the file descriptor using string stream
            pipeNumberStr = oss.str(); // Getting the file descriptor as an int

            // This calls the ROS module "senddata" (located in data_from_yarp package) which publishes the data on a ROS topic
            std::exit(EXIT_SUCCESS);          
            execlp("rosrun", "rosrun", "data_from_yarp", "senddata", pipeNumberStr.c_str(), NULL );
            cerr << "rosrun could not be launched : " << strerror(errno) << endl;
            killFatherAndExit(); // Abort
            break;                
        default: // Parent process of the fork()
            break;
    }
}    

void openRosReceiver()
{
    switch (fork()) 
    {
        case -1 : 
            cerr << "Fork instruction has failed (receiver) : " << strerror(errno) << endl;
            std::exit(EXIT_FAILURE);
    
        case 0 : 
            // replace receiver_image_ros with the module taking the image
            std::exit(EXIT_SUCCESS);
            execlp("rosrun","rosrun","data_from_yarp","receivedata", NULL );
            cerr << "rosrun could not be launched : " << strerror(errno) << endl;
            killFatherAndExit(); // Abort
             
        default:
            break;
    }
}

void killFatherAndExit()
{
    kill(getppid(),SIGTERM); // Killing the father process
    std::exit(EXIT_FAILURE); // Suicide
}




