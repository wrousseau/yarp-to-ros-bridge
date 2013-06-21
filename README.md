    __   __ ___  ______ ______  ______  _____  _____  ______ ______  _____ ______  _____  _____ 
    \ \ / // _ \ | ___ \| ___ \ | ___ \|  _  |/  ___| | ___ \| ___ \|_   _||  _  \|  __ \|  ___|
     \ V // /_\ \| |_/ /| |_/ / | |_/ /| | | |\ `--.  | |_/ /| |_/ /  | |  | | | || |  \/| |__  
      \ / |  _  ||    / |  __/  |    / | | | | `--. \ | ___ \|    /   | |  | | | || | __ |  __| 
      | | | | | || |\ \ | |     | |\ \ \ \_/ //\__/ / | |_/ /| |\ \  _| |_ | |/ / | |_\ \| |___ 
      \_/ \_| |_/\_| \_|\_|     \_| \_| \___/ \____/  \____/ \_| \_| \___/ |___/   \____/\____/ 
                                                                                                
                                                                                                
                    _____  _____  _   _  _____ ______   ___  _____  _____ ______ 
                   |  __ \|  ___|| \ | ||  ___|| ___ \ / _ \|_   _||  _  || ___ \
                   | |  \/| |__  |  \| || |__  | |_/ // /_\ \ | |  | | | || |_/ /
                   | | __ |  __| | . ` ||  __| |    / |  _  | | |  | | | ||    / 
                   | |_\ \| |___ | |\  || |___ | |\ \ | | | | | |  \ \_/ /| |\ \ 
                    \____/\____/ \_| \_/\____/ \_| \_|\_| |_/ \_/   \___/ \_| \_|

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

========= FINAL PURPOSE ========
This bridge will allow generic communication between YARP and ROS.

======== HOW TO COMPILE/INSTALL ========
If you followed the installation instructions for MACSI and if you keep the paths
intacts, you only have to add $MACSI_ROOT to your $ROS_PACKAGE_PATH. 
You may want to do that in your .bashrc/.zshrc so that you can use ROS companions functions
on the topics created by the bridge :

> echo "export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$MACSI_ROOT" >> ~/.bashrc # If you use Bash obviously

The only thing needed to be done is to enter "yarp_bridge" and do the following :

> mkdir build
> cd build
> cmake ..
> make

The single executable file is currently name "yarpbridge" and will be located in "yarp_bridge/bin".
This can be cumbersome but this allows bin to contain all useful files and "build" to contain all CMake files which are pretty much useless.
In order to not have to move from "build" to "bin", a "make run" from the "build" folder will allow you to launch the program.

======== HOW TO USE ===========
The program currently supports communication between a ROS topic and a YARP port, on both sides.
It succesfully sends any type of data, including image.
Please enter the configuration (module name, ports you want to use) in the .ini file located in :
$MACSI_ROOT/main/app/ros/bridge.ini

You can also pick a custom ".ini" file following the same format by using the option :
./yarpbridge --from custom.ini
Two examples (bridge_data.ini and bridge_image.ini) are provided in the ini directory.
For more informations about custom .ini files, please check the YARP documentation dealing with ResourcesFinder.

You can also use the GUI tool in order to generate such configurations files, save them or open previous ones.

======= YARP->ROS ==========
The entry is a YARP port and the data is processed all the way to a ROS topic. 
Thus, the YARP port specified in the .ini file must be the port you're trying to get data from. 
You can pick any name for the ROS topic, just make sure it does not exist yet.

The bridge processes the .ini file, figures out what types need to be used in ROS and create a custom .msg file.
It then has to cmake the ROS part of the bridge to generate the headers corresponding to the .msg file.
It then compiles the ROS part of the bridge.
For that reason, using this direction for the bridge can be quite long, though it checks if the .ini file was modified before recompiling.
It then launches the ROS part of the bridge and sends data through the pipe.
For images, generating the .msg file is not needed but the ROS node will be recompiled.

Automatic mode allows the user to not enter the types and names of data sent. It however right now works mostly with vectors of simple data
and not complexe structures. Give it a try, and if it does not work, check documentation to see what precisely stands in the YARP port.

======= YARP->ROS ==========
The entry is a ROS topic and the data is processed all the way to a YARP port. 
Thus, the ROS topic specified in the .ini file must be the topic you're trying to get data from. 
You can pick any name for the YARP port, just make sure it does not exist yet.

The bridge processes the .ini file and uses a Python wrapper to grab messages from the ROS topic as a string.
It sends these strings through a pipe.
A child process receives these strings and processes them in YARP bottles before sending these bottles on the YARP port.
When dealing with images, the Python wrapper is not needed as it is only used because we don't know anything about the message before run-time.

Automatic mode allows the user to not enter the types and names of data sent. It works using a Python script but the topic you specified
in the .ini file must exist and the ROS msg which specifies the data living in that topic must be registered. Give it a try, and if it does not work, this command should give you precisely what's in this topic :
> rostopic type $rosTopic$ | rosmsg show

======= TO DO ==============
> (YARP->ROS) Storing the ROS node dealing with images and copy it when needed instead of recompiling it. 
> (ROS->YARP) Dealing with less classic ROS types such as Time, Duration, Array Types and Headers
> Processes are sometimes not terminated in a clean way. For some reason, YARP sometimes does not like SIGINT signals.

======= WILL RUN ON ========================
The application was tested on Ubuntu Precise (12.04) with ROS Groovy.
