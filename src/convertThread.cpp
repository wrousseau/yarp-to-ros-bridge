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
 * \file convertThread.cpp
 * \brief ConvertThread class methods
 * \author Woody Rousseau
 * \version 0.1
 * \date 23/05/13
 */

#include <yarp/os/Network.h>

#include "convertThread.h"

FILE* fromYarpPipe; // In order to deal with file descriptors, we must use a C FILE*
bool loopAgain = false;

ConvertThread::ConvertThread(string _name, string _yarpPort, string _rosTopic, int _rate, int *_fd) : 
    RateThread(_rate), name(_name), yarpPort(_yarpPort), rosTopic(_rosTopic), firstData(true), fd(_fd)
{
    sigemptyset( &sigact.sa_mask );
    sigact.sa_handler = catcher;
    sigact.sa_flags = 0;
    if (sigaction( SIGUSR1, &sigact, NULL ) == -1)
        cerr << "sigaction" << endl;

}

void ConvertThread::openRosSender()
{
    switch ( childPid = fork() ) 
    {
        case -1 : // Something went wrong. Abort.
            cerr << "fork" << endl;
    
        case 0 : // Child process of the fork()
            close(fd[P_WRITE]);
            launchRosModule();          
            break; // The child process never goes there (execlp or error)                
        default: // Parent process of the fork()
            break;
    }

}    

void ConvertThread::launchRosModule()
{
    string readFD = to_string(fd[P_READ]);
    cout << "Launching ROS module..." << endl;
    execlp("./rosreception","./rosreception", readFD.c_str(), rosTopic.c_str(), to_string(getppid()).c_str(), NULL );
    cerr << "what the hell are we doing here ?" << endl;
}

void catcher (int sig)
{
    loopAgain = true;
}