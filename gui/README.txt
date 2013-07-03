
    __   __ ___  ______ ______  ______  _____  _____  ______ ______  _____ ______  _____  _____ 
    \ \ / // _ \ | ___ \| ___ \ | ___ \|  _  |/  ___| | ___ \| ___ \|_   _||  _  \|  __ \|  ___|
     \ V // /_\ \| |_/ /| |_/ / | |_/ /| | | |\ `--.  | |_/ /| |_/ /  | |  | | | || |  \/| |__  
      \ / |  _  ||    / |  __/  |    / | | | | `--. \ | ___ \|    /   | |  | | | || | __ |  __| 
      | | | | | || |\ \ | |     | |\ \ \ \_/ //\__/ / | |_/ /| |\ \  _| |_ | |/ / | |_\ \| |___ 
      \_/ \_| |_/\_| \_|\_|     \_| \_| \___/ \____/  \____/ \_| \_| \___/ |___/   \____/\____/ 
                                                                                                
                                                                                                
      _____  _____  _   _  _____ ______   ___  _____  _____ ______         _____  _   _  _____ 
     |  __ \|  ___|| \ | ||  ___|| ___ \ / _ \|_   _||  _  || ___ \       |  __ \| | | ||_   _|
     | |  \/| |__  |  \| || |__  | |_/ // /_\ \ | |  | | | || |_/ /______ | |  \/| | | |  | |  
     | | __ |  __| | . ` ||  __| |    / |  _  | | |  | | | ||    /|______|| | __ | | | |  | |  
     | |_\ \| |___ | |\  || |___ | |\ \ | | | | | |  \ \_/ /| |\ \        | |_\ \| |_| | _| |_ 
      \____/\____/ \_| \_/\____/ \_| \_|\_| |_/ \_/   \___/ \_| \_|        \____/ \___/  \___/ 
                                                                                               
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

################ PURPOSE ################################################
This gui allows the user to configure the bridge using a GUI application, without manually editing
the .ini file. It then basically launches the YARP ROS BRIDGE GENERATOR using the current
configuration displayed in the GUI.
The GUI allows the user to save configurations, open previously created ones.
The console output is displayed once the "Launch" button is pressed on a second window.

############### COMPILE #################################################
Compilation is possible using qmake as the project was made using QT 4.8. Because it is not 
always installed and requires a somewhat heavy SDK (Qt's), we have included the executable file
in the bin folder (yarpros_bridge_generator/bin). Sources are available and compilation is possible using :
> mkdir build
> cd build
> qmake ..
> make

############## RUN ######################################################
Launch the GUI from the command line or through the file explorer.

