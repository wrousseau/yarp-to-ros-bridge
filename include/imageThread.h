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
#include <yarp/sig/Image.h>
// Opencv Libraries
#include <opencv/cv.h>
#include <opencv/highgui.h> 

#include "bridge.h"
#include "convertThread.h"
#include "bridgeHeader.h"

using namespace yarp::sig;
using namespace cv;

extern FILE* fromYarpPipe; /**< We get that global FILE* because it is here that we write in it ! */

/** \class ImageThread
   * \brief Sending and image and its headers through the pipe
   *
   * This class does almost all the work ! It opens the device port, let us connect with it
   * It then converts the yarp image into something opencv can handle, processes the headers before sending it through the pipe
   */
class ImageThread: public ConvertThread
{

private:
	int width; /**< Width of the image */
	int height; /**< Height of the image */
	IplImage *image8u; /**< IplImage which is basically what we get from the camera */
	Mat matImage; /**< The Mat format allows us to send it to ROS which uses opencv */
	BufferedPort<ImageOf<PixelRgb>> imagePort; /**< The port connected to the device port to gather data from the camera */

public:
	/**
     *  \brief Constructor
     *
     *  Construct a ImageThread from the parameters gathered in ConvertModule
     *
     * \param _name : The name of the module
     * \param _yarpPort : The YARP port from which we will be getting the images data
     * \param _rosTopic : The ROS topic on which we want to publish the images
     * \param _rate : The rate of the thread which is needed by the parent constructor
     * \param _fd : Pointer to file descriptors
     */
	ImageThread(string _name, string _yarpPort, string _rosTopic, int _rate, int* _fd);

	/**
     * \brief Initializing the thread
     *
     * The thread executes this function when it starts and before "run". 
     * The return value of threadInit() is notified to the class and passed as a parameter to afterStart()
     *
     * \return If the function returns false the thread quits and never calls "run"
     */
	virtual bool threadInit();

    /**
     *  \brief Thread running at rate period
     *
     *  Does all the work, which is sending the image and its headers through the pipe to the other middleware
     */
	virtual void run();

	/**
     *  \brief Releasing the thread
     *
     *  Mostly closing the client port
     */
 	virtual void threadRelease();

 };

 #endif