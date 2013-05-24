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
 * \file bridgeHeader.cpp
 * \brief BridgeHeader functions
 * \author Woody Rousseau
 * \version 0.1
 * \date 23/05/13
 */

#include "bridgeHeader.h"

void readBridgeHeaderVector(Bottle &rf, string name, vector<BridgeHeader> &v, int size)
{
    v.reserve(size); // Let's make sure there's just enough room for our headers
    if ( rf.check( name.c_str() ) )
    {
        Bottle &grp = rf.findGroup(name.c_str());
        for ( int i = 0; i < size; i++)
        {
            BridgeHeader data = { grp.get(1+i).asString().c_str() }; // Just initializing the name, we don't yet know the type
            v.push_back(data);
        }
    }
    else
    {
        cout << "Could not find parameters for " << name << ". "
            << "Setting everything to null by default" << endl;
    }
}