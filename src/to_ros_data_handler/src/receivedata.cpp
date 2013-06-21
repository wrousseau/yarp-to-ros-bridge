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
 * \file receivedata.cpp
 * \brief Sending data received from the bridge to a ROS topic
 * \author Woody Rousseau
 * \version 0.1
 * \date 29/05/13
 */

#include "ros/ros.h"
#include "to_ros_data_handler/Data.h"
#include "generatedGetter.h"
#include <sstream>
#include <iostream>
#include <string>
#include <cstdio>
#include <signal.h>

using namespace std;

int main(int argc, char **argv)
{
    // Setting up ROS
    ros::init(argc, argv, "rosreception");
    ros::NodeHandle n; 
    ros::Publisher chatterPub = n.advertise<to_ros_data_handler::Data>(argv[2], 1000);
    ros::Rate loopRate(1000);

    char text[1000];
    string data;
    FILE* pipe = NULL; // File Pipe
    pipe = fdopen(atoi(argv[1]),"r");
    if (pipe == NULL)
        cerr << "fdopen, Read" << endl;

    pid_t fatherPid = atoi(argv[3]);

    while (ros::ok())
    {
        to_ros_data_handler::Data msg;
        // Getting the data
        if (fgets(text,1000,pipe) == NULL)
        {
            cerr << "[ROS]::fgets" << endl;
        }

        kill(fatherPid,SIGUSR1);

        data = text;

        getNames(msg,data);

        // Sending the data  
        chatterPub.publish(msg);

        ros::spinOnce();

        loopRate.sleep();
    }
    

    exit(EXIT_SUCCESS);
}

