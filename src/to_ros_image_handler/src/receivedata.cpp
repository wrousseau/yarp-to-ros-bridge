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
 * \brief Sending image data received from the bridge to a ROS topic
 * \author Meriem Guenaien
 * \version 0.1
 * \date 29/05/13
 */

#include "rosBridgeImage.h"


int main(int argc, char **argv)
{
    // Setting up Ros
    ros::init(argc, argv, "rosreception");
    ros::NodeHandle n;

    // Replace image_discussion with the name of the topic used to read images
    ros::Publisher chatterPub = n.advertise<sensor_msgs::Image>(argv[2], 10);

    ros::Rate loopRate(10);

    // Received Image
    Mat receivedImage;
    int rows,cols;
    int numDescp;
    char text[200];
    FILE* pipe =NULL;
    numDescp=atoi(argv[1]);
    cv_bridge::CvImage cvImageOut;

    if( (pipe = fdopen(numDescp,"r"))==NULL)
    { 
        cout << "[ROS]:::error in receiving" << endl;
        exit(EXIT_FAILURE);
    }

    pid_t fatherPid = atoi(argv[3]);

    while (ros::ok())
    { 
        fd_set rset ;
        FD_ZERO(&rset);
        FD_SET( numDescp,&rset);

        if( select ( numDescp+1 , &rset, NULL, NULL, NULL ) < 0 )
        {
            cerr << "[ROS]:::error on select" << endl;
            return(EXIT_FAILURE);
        }
        if (!FD_ISSET(numDescp,&rset))
        {
            cerr << "[ROS]:::no one is set"<<endl;
            return(EXIT_FAILURE) ;
        }

        //reading columns and rows
        if (fgets(text,100,pipe) == NULL)
        {
            cerr << "Fgets rows" << endl;
            exit(EXIT_FAILURE);
        }
        rows = atoi(text);
        if (fgets(text,100,pipe) == NULL)
        {
            cerr << "Fgets columns" << endl;
            exit(EXIT_FAILURE);
        }
        cols = atoi(text);

        //reading the image
        receivedImage.create(rows,cols,CV_8UC3);

        int size = rows*cols*3;  

        if ( fread( receivedImage.data, sizeof(unsigned char), size, pipe ) == 0 )
        {
            cerr << "[ROS]:::error in reading the image : " << endl;
            return(EXIT_FAILURE);
        }
                    
        // Using cv bridge to convert the image from char* to cv::mat image
        cvImageOut.image=receivedImage;
        cvImageOut.encoding=enc::BGR8;

        kill(fatherPid,SIGUSR1);

        //Publishing the image
        chatterPub.publish(cvImageOut.toImageMsg());

        receivedImage.release();
        cvImageOut.image.release();
  
        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
}
