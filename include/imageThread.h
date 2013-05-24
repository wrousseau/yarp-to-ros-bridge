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
 * \file imageThread.h
 * \brief ImageThread class
 * \author Woody Rousseau
 * \version 0.1
 * \date 23/05/13
 *
 * The thread sends/receives an image and its headers through the pipe using opencv
 */

#ifndef IMAGE_THREAD_H
#define IMAGE_THREAD_H

// Yarp Libraries
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Image.h>
// Opencv Libraries
#include <opencv/cv.h>
#include <opencv/highgui.h> 

#include "dataToRos.h"
#include "convertThread.h"
#include "bridgeHeader.h"

using namespace yarp::sig;
using namespace cv;

extern FILE* yarpPipe; /**< We get that global FILE* because it is here that we write in it ! */

/** \class ImageThread
   * \brief Sending and image and its headers through the pipe
   *
   * This class does almost all the work ! It opens the device port, let us connect with it
   * It then converts the yarp image into something opencv can handle, processes the headers before sending it through the pipe
   */
class ImageThread: public ConvertThread
{

private:
	int width;
	int height;
	IplImage *image8u;
	Mat matImage;
	BufferedPort<ImageOf<PixelRgb>> imagePort;
public:
	ImageThread(string _name, string _robot, string _devicePort, int _rate);
	virtual bool threadInit();
	virtual void run();
 	virtual void threadRelease();

 };

 #endif