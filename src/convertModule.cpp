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
 * \file convertModule.cpp
 * \brief ConvertModule class methods
 * \author Woody Rousseau
 * \version 0.1
 * \date 23/05/13
 */

#include <yarp/os/Time.h>

#include "convertModule.h"
#include "dataThread.h"
#include "imageThread.h"

bool ConvertModule::configure(yarp::os::ResourceFinder &rf)
{
    Time::turboBoost(); // For OS where it makes sense, sets the scheduler to be called more often

    cout << "Reading parameters from .ini file... " << endl;

    // General part
    //....................................................
    Bottle &bGeneral = rf.findGroup("general");
    readString( bGeneral, "name", name, "bridgegenerator" );
    readString( bGeneral, "bridgetype", bridgeType, "yarp2ros" );
    readInt( bGeneral, "datatype", dataType, 0);
    readInt( bGeneral, "numberofyarpgroups", numberOfYarpGroups, 0 );
    readBridgeHeaderVector( bGeneral, "yarpgroups", yarpGroups, numberOfYarpGroups );
    readInt( bGeneral, "numberofrosgroups", numberOfRosGroups, 0 );
    readBridgeHeaderVector( bGeneral, "rosgroups", rosGroups, numberOfRosGroups );
    readInt( bGeneral, "rate", rate, 10 );
    
    // Yarp part
    //....................................................
    Bottle &bYarp=rf.findGroup("yarp");
    readString( bYarp, "robot", robot, "icubSim" ); //by default simulator, so we dont break the real one
    readString( bYarp, "port", devicePort, "/read" );

    for ( int i = 0 ; i < numberOfYarpGroups ; i++)
    {
        string groupName = "yarp_" + yarpGroups[i].name;
        Bottle &bYarpGroup=rf.findGroup(groupName.c_str());
        readString(bYarpGroup, "type", yarpGroups[i].type, "float");
    }


    // Ros part
    //....................................................
    Bottle &bRos=rf.findGroup("ros");

    for (int i = 0 ; i < numberOfRosGroups ; i++)
    {
        string groupName = "ros_" + rosGroups[i].name;
        Bottle &bRosGroup=rf.findGroup(groupName.c_str());
        readString(bRosGroup, "type",rosGroups[i].type, "float");
    }

    // Launching the thread
    //....................................................
    switch (dataType)
    {
        case 0:
            thread = new DataThread(name, robot, devicePort, yarpGroups, rate);
            break;
        case 1:
            thread = new ImageThread(name, robot, devicePort, rate);
            break;
        default:
            cerr << "The .ini file does not properly gives the data type (0 = data, 1 = image)" << endl;
            return false;
    }
    if ( !thread->start() )
    {
        cout << "[yarp]::Error : the thread could not start" << endl;
        delete thread;    
        return false;
    }

    // Opening the rpc port so that we can send rpc commands
    //....................................................
    string port = "/" + name + "/rpc";
    rpcPort.open(port.c_str());
    attach(rpcPort);
    return true;
}

bool ConvertModule::close()
{
    rpcPort.interrupt();
    rpcPort.close();
    thread->stop();
    delete thread;
    return true;
}

double ConvertModule::getPeriod()
{ 
    return 1.0;  
}

bool ConvertModule::updateModule() 
{
    return true; 
}
