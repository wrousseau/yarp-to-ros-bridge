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
 * \file rosBridgeImage.h
 * \brief Libraries and namespaces to handle images
 * \author Woody Rousseau
 * \version 0.1
 * \date 29/05/13
 *
 */

#ifndef H_ROS_BRIDGE_IMAGE
#define H_ROS_BRIDGE_IMAGE

#include "ros/ros.h"
#include <sstream>
#include <iostream>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <sys/select.h>
#include <signal.h>


#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;
namespace enc = sensor_msgs::image_encodings;
using namespace cv_bridge;

#endif
