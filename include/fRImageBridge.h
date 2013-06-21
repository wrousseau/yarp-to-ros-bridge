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
 * \file fRImageBridge.h
 * \brief Sending image data from a ROS topic to a YARP port
 * \author Woody Rousseau
 * \version 0.1
 * \date 05/06/13
 */

#ifndef F_R_IMAGE_BRIDGE_H
#define F_R_IMAGE_BRIDGE_H

// STL
#include <functional>
// YARP
#include <yarp/sig/Image.h>
// ROS
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
// OpenCv
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
// Cv_bridge
#include <cv_bridge/cv_bridge.h>
// Project headers
#include "fRBridge.h"
// Namespaces
using namespace cv;
namespace enc = sensor_msgs::image_encodings;
using namespace yarp::sig;

/** \class FRImageBridge
   * \brief Child class to deal with images from a ROS topic to a YARP port
   *
   * No Python script is used here
   */
class FRImageBridge : public FRBridge
{
private:
    FILE* fromRosPipe; /**< Pipe file */
    bool firstData; /**< Is this the first data sent through the pipe ? Used to avoid assigning width and height at each loop run */
    BufferedPort<ImageOf<PixelRgb>> imagePort; /**< The port connected to the device port to gather image from the camera */

public:
        /**
     *  \brief Constructor
     *
     * \param _fd : Pointer to the file descriptors
     * \param _topicName : Name of the ROS topic (used as input)
     * \param _yarpPort : Name of the YARP port (used as output)
     */
    FRImageBridge(int* _fd, string _topicName, string _yarpPort);

    /**
     *  \brief Virtual method which launches the ROS node with the writing callback method
     *
     * \return True/False upon success/Failure but won't normally return by itself
     *
     * Launches a ROS node which calls a callback method
     */
    virtual bool writeData();

    /**
     *  \brief Virtual method which handles reading the image on the pipe
     *
     * \return True/False upon success/Failure but won't normally return by itself
     *
     * Launches a Python scripts which grabs at a certain rate a message from the ROS topic, creates a string out of it, and sends it through the pipe
     */
    virtual bool readRosData();

    
    /**
     *  \brief Callback method which handles writing the image on the pipe
     *
     * \param msg A ROS ImageConstrPtr message
     *
     * Grabs an image in CvImageConstPtr format from the ROS topic, writes its data, width and height on the pipe
     */
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

};

#endif
