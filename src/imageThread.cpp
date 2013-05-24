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
 * \file imageThread.cpp
 * \brief ImageThread methods
 * \author Woody Rousseau
 * \version 0.1
 * \date 23/05/13
 */

#include "imageThread.h"

ImageThread::ImageThread(string _name, string _robot, string _devicePort, int _rate) : 
    ConvertThread(_name, _robot, _devicePort, _rate), width(0), height(0), image8u(NULL)
{
}

bool ImageThread::threadInit()
{
    string clientPort = "/" + name + "/img:i";
    
    string type="udp";

    // Open the port for receiving the images
    imagePort.open(clientPort.c_str()); 
    // Cnnecting the port to the icub camera 
    Network::connect(("/"+robot+devicePort).c_str(),clientPort.c_str(),type.c_str(),false);
    return true;
}

void ImageThread::threadRelease()
{
    imagePort.interrupt();
    imagePort.close();
    image8u = 0;
    ConvertThread::threadRelease();
}

void ImageThread::run()
{
    //read an image     
    ImageOf<PixelRgb> *yarpImage = imagePort.read(); 

    //if the port actually received an image message
    if (yarpImage!=NULL) 
    { 
        if(firstData)
        {
            // Get image dimensions
            width = yarpImage->width();
            height = yarpImage->height();
            
            // Create opencv image with the same dimensions
            image8u = cvCreateImage(cvSize(yarpImage->width(), yarpImage->height()), IPL_DEPTH_8U, 3);
            matImage.create( yarpImage->width(), yarpImage->height(), CV_8UC3 );
            firstData = false;
            
            cout << "[yarp]:::Images sent have size: w=" << width << " h=" << height << endl;
        }

        // Convert the image form IPLImage to cv::Mat in order to be sent by the pipe
        matImage = Mat(static_cast<IplImage*>(yarpImage->getIplImage()));   
        cvtColor(matImage, matImage, CV_BGR2RGB);
        
        // Display the image sent by yarp
        namedWindow( "yarp_image", CV_WINDOW_AUTOSIZE );
        imshow( "yarp_image",matImage);    
        waitKey(100);

        // Sending the image by the pipe
        fprintf(yarpPipe,"image\n");
        fprintf(yarpPipe,"%d\n",matImage.rows);
        fprintf(yarpPipe,"%d\n",matImage.cols);
            
        int size = matImage.rows*matImage.cols*3;
        if (fwrite(matImage.data,sizeof(unsigned char),size,yarpPipe) == 0)
        {
            cerr << "[yarp]::Sending error : " << strerror(errno) << endl;
        }
        fflush(yarpPipe);
        matImage.release();
    }
}