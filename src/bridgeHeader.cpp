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

void readBridgeHeaderVector(Bottle &rf, string name, field &groups, int size)
{
    groups.clear();
    if ( rf.check( name.c_str() ) )
    {
        Bottle &grp = rf.findGroup(name.c_str());
        for ( int i = 0; i < size; i++)
        {
            groups.push_back({grp.get(1+i).asString().c_str(),""});
        }
    }
    else
    {
        cout << "Could not find parameters for " << name << ". "
            << "Setting everything to null by default" << endl;
    }
}

void generateRosMessage(const field &groups)
{
    mkdir("../src/to_ros_data_handler/msg",0775);
    ofstream msgFile("../src/to_ros_data_handler/msg/Data.msg");
    if (msgFile.is_open())
    {
        for ( auto &header : groups)
        {
            msgFile << convertToRos(header.second) << " " << header.first << endl;
        }
    }
    mkdir("../src/to_ros_data_handler/include",0775);
    ofstream getter("../src/to_ros_data_handler/include/generatedGetter.h");
    if (getter.is_open())
    {
        getter << "#include <string>" << endl << "#include <sstream>" << endl << "#include \"to_ros_data_handler/Data.h\"" << endl;
        getter << "using namespace std;" << endl;
        getter << "void getNames(to_ros_data_handler::Data &msg, string data)" << endl << "{" << endl;
        getter << "stringstream in(data);" << endl;
        getter << "in ";
        for (auto &header : groups)
        {
            if (header.second == "bool")
                getter << " >> std::boolalpha";
            getter << " >> msg." << header.first;
        }
        getter << ";" << endl;
        getter << "}"; 
    }
}

string convertToRos(string type)
{
    vector<string> rosTypes = {"bool","int8","uint8","int16","uint16","int32","uint32","int64","uint64","float32","float64","string","time","duration"};
    if (std::find(rosTypes.begin(), rosTypes.end(), type) != rosTypes.end())
    {
        return type;
    }
    string result;
    if (type == "char")
    {
        result = "int8";
    }
    else if (type == "unsigned char")
    {
        result = "uint8";
    }
    else if (type == "short" || type == "short int")
    {
        result = "int16";
    }
    else if (type == "long" || type == "long int")
    {
        result = "int32";
    }
    else if (type == "unsigned long" || type == "unsigned long int")
    {
        result = "uint32";
    }
    else if (type == "long long" || type == "long long int")
    {
        result = "int64";
    }
    else if (type == "unsigned long long" || type == "unsigned long long int")
    {
        result = "uint64";
    }
    else if (type == "unsigned short" || type == "unsigned short int")
    {
        result = "uint16";
    }
    else if (type == "float" || type == "int" || type == "uint")
    {
        result = type + "32";
    }
    else if (type == "unsigned" || type == "unsigned int")
    {
        result = "uint32";
    }
    else if (type == "double" || type == "long double")
    {
        result = "float64";
    }
    else
    {
        cout << "The type \"" << type << "\" you entered in the .ini file is probably not recognized by ROS" << endl;
        cout << "Types supported : float, int, uint, unsigned, unsigned int, bool, int8, uint8, int16, uint16, int32, int64, uint64, float32, float64, string, time, duration" << endl;
    }
    return result;
}
