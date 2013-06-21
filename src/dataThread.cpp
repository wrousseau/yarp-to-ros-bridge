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

#include "dataThread.h"

extern bool loopAgain;

DataThread::DataThread(string _name, string _yarpPort, string _rosTopic, field _yarpGroups, int _rate, int *_fd) : 
    ConvertThread(_name,_yarpPort, _rosTopic, _rate, _fd), yarpGroups(_yarpGroups)
{

}

bool DataThread::threadInit()
{       
    string clientPort = "/" + name;
    
    string type="tcp";

    // Open the port for receiving the images
    inPort.open(clientPort.c_str()); 
    // Cnnecting the port to the icub camera 
    Network::connect((yarpPort).c_str(),clientPort.c_str(),type.c_str(),false);
    ConvertThread::openRosSender();
    close(fd[P_READ]);

    // Only the original process (father) has survived, he's the one which will be writing, so we don't need the "read" side    
    fromYarpPipe = fdopen(fd[P_WRITE],"w");
    if (fromYarpPipe == NULL)
    {
        cerr << "Fdopen, Write" << endl;
        std::exit(EXIT_FAILURE);
    }
    return true;
}

void DataThread::threadRelease()
{
    cout << "Releasing the Thread" << endl;
    kill(childPid,SIGTERM);
    inPort.interrupt();
    inPort.close();
}

void DataThread::run()
{
    string data;
    // Reading the data from the client port     
    Bottle* bot = inPort.read();
    // If the port actually received data
    if (bot != NULL) 
    { 
        data = bot->toString();
        loopAgain = false;
        //cout << "[yarp]:::I received " << bot->toString() << endl;
        fprintf(fromYarpPipe,"%s\n",data.c_str()); // Sending the data itself through the pipe
        //cout << "[yarp]:::I sent " << headers << data << endl;
        fflush(fromYarpPipe); // Flushing the pipe to prepare for the next message
        while (!loopAgain)
        {}    
    }
}


  
