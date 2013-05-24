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

#include "convertThread.h"

ConvertThread::ConvertThread(string _name, string _robot, string _devicePort, int _rate) : 
    RateThread(_rate), name(_name), robot(_robot), devicePort(_devicePort), firstData(true)
{
}
