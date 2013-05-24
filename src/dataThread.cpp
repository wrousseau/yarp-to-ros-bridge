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

DataThread::DataThread(string _name, string _robot, string _devicePort, vector<BridgeHeader> _yarpGroups, int _rate) : 
    ConvertThread(_name,_robot,_devicePort,_rate), yarpGroups(_yarpGroups)
{
}

bool DataThread::threadInit()
{       
    string clientPort = "/" + name;
    
    string type="tcp";

    headers = getDataHeadersString();

    // Open the port for receiving the images
    inPort.open(clientPort.c_str()); 
    // Cnnecting the port to the icub camera 
    Network::connect(("/"+robot+devicePort).c_str(),clientPort.c_str(),type.c_str(),false);
    return true;
}

void DataThread::threadRelease()
{
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
        if (firstData)
        {
            data = bot->toString();
            cout << "[yarp]:::I received " << bot->toString() << endl;
        }
        
        fprintf(yarpPipe,"%s",headers.c_str()); // Sending the headers through the pipe (types and names) as a string
        fprintf(yarpPipe,"%s\n",data.c_str()); // Sending the data itself through the pipe
        cout << "[yarp]:::I sent " << headers << data << endl;
        fflush(yarpPipe); // Flushing the pipe to prepare for the next message
    }
}

string DataThread::getDataHeadersString()
{
    string dataHeadersString = "";
    for (auto i : yarpGroups)
    {
        dataHeadersString += i.type + " " + i.name + " ";
    }
    return dataHeadersString;
}
