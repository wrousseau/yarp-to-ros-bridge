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
 * \file fRImageBridge.cpp
 * \brief Sending image data from a ROS topic to a YARP port
 * \author Woody Rousseau
 * \version 0.1
 * \date 05/06/13
 *
 * Uses a Python wrapper
 */


#include "fRImageBridge.h"

FRImageBridge::FRImageBridge(int* _fd, string _topicName, string _yarpPort) : FRBridge(_fd,_topicName,_yarpPort), fromRosPipe(NULL), firstData(true)
{
}

bool FRImageBridge::writeData()
{
    close(fd[P_READ]);
    if ( (fromRosPipe = fdopen(fd[P_WRITE],"w")) == NULL )
    {
        cerr << "Fdopen (Write)" << endl;
        return false;
    }
    int argc = 0;
    char **argv = NULL;
    ros::init(argc,argv,"rossender");

    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe(topicName, 10, &FRImageBridge::imageCallback,this);
    ros::spin();
    return true;
}

void FRImageBridge::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        if (enc::isColor(msg->encoding))
            cv_ptr = cv_bridge::toCvShare(msg, enc::BGR8);
        else
            cv_ptr = cv_bridge::toCvShare(msg, enc::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    fprintf(fromRosPipe,"%d\n",cv_ptr->image.rows);
    fprintf(fromRosPipe,"%d\n",cv_ptr->image.cols);
            
    int size = cv_ptr->image.rows*cv_ptr->image.cols*3;
    if ( fwrite( cv_ptr->image.data, sizeof(unsigned char), size, fromRosPipe) == 0 )
    {
        cerr << "[yarp]::Sending error : " << strerror(errno) << endl;
    }
    fflush(fromRosPipe);
}

bool FRImageBridge::readRosData()
{
    close(fd[P_WRITE]);
    if ( (fromRosPipe = fdopen(fd[P_READ],"r")) == NULL )
    {
        cerr << "Fdopen (Read)" << endl;
        return false;
    }
    // Open the port for receiving the images
    imagePort.open(yarpPort.c_str());
    ImageOf<PixelRgb> yarpImage = imagePort.prepare();
    Mat receivedImage;
    int rows, cols, size;

    char rowsC[10];
    char colsC[10];

    while (true)
    {
         //reading columns and rows
        if (fgets(rowsC,100,fromRosPipe) == NULL)
        {
            cerr << "Fgets rows" << endl;
            exit(EXIT_FAILURE);
        }
        if (fgets(colsC,100,fromRosPipe) == NULL)
        {
            cerr << "Fgets columns" << endl;
            exit(EXIT_FAILURE);
        }
        if (firstData)
        {
            rows = atoi(rowsC);
            cols = atoi(colsC);
            size = rows*cols*3;
            firstData = false;
        }

        //reading the image
        receivedImage.create(rows,cols,CV_8UC3);

        if ( fread( receivedImage.data, sizeof(unsigned char), size, fromRosPipe ) == 0 )
        {
            cerr << "[ROS]:::error in reading the image : " << endl;
            return(EXIT_FAILURE);
        }

        IplImage cvImage = receivedImage;
        yarpImage.wrapIplImage(&cvImage);
        receivedImage.release();

        imagePort.write();
    }
}