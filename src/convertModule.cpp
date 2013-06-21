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
 * \file convertModule.cpp
 * \brief ConvertModule class methods
 * \author Woody Rousseau
 * \version 0.1
 * \date 23/05/13
 */
#include <Python.h>

#include <yarp/os/Time.h>

#include "convertModule.h"
#include "dataThread.h"
#include "imageThread.h"
#include "fRDataBridge.h"
#include "fRImageBridge.h"

ConvertModule::ConvertModule(int *fd, bool gui) : fd(fd), isGui(gui), count(0)
{}

bool ConvertModule::configure(yarp::os::ResourceFinder &rf)
{
    Time::turboBoost(); // For OS where it makes sense, sets the scheduler to be called more often

    cout << "Reading parameters from .ini file... " << endl;
    
    string lastActualEdit;
    bool updated = checkUpdate(rf, lastActualEdit); // Checking if the .ini file has changed

    // Yarp part
    //....................................................
    Bottle &bYarp=rf.findGroup("yarp");
    readString( bYarp, "port", yarpPort, "/bridge_port" );

    // ROS part
    //....................................................
    Bottle &bRos=rf.findGroup("ros");
    readString(bRos,"topic",rosTopic,"/bridge_topic");

    // General part
    //....................................................
    Bottle &bGeneral = rf.findGroup("general");
    readString( bGeneral, "name", name, "bridgegenerator" );
    readString( bGeneral, "bridgetype", bridgeType, "yarp2ros" );
    readInt( bGeneral, "rate", rate, 10 );
    readInt( bGeneral, "datatype", dataType, 0);
    if (!isGui && !checkPort(yarpPort,dataType))
    {
        close();
        return true;
    } 
    if (!dataType)
    {
        readInt( bGeneral, "automatic", automatic, 0);
        if (automatic && !tryAutomaticMode())
        {
            close();
            return true;
        }
        if (!automatic) // We only consider those fields if we're sending classic data
        {
            readInt( bGeneral, "numberofgroups", numberOfGroups, 0 );
            readBridgeHeaderVector( bGeneral, "groups", groups, numberOfGroups );
            for (auto &header : groups )
            {
                string groupName = "data_" + header.first;
                Bottle &bGroup=rf.findGroup(groupName.c_str());
                readString(bGroup, "type", header.second, "float");
            }
        }
    }

    if (bridgeType == "ros2yarp")
    {
        switch (dataType)
        {
            case 0:
                fRBridge.reset(new FRDataBridge(fd, rosTopic, yarpPort, groups));
                break;
            case 1:
                fRBridge.reset(new FRImageBridge(fd, rosTopic, yarpPort));
                break;
            default:
                cerr << "The .ini file does not properly gives the data type (0 = data, 1 = image)" << endl;
                close();
                return false;
        }
        fRBridge->run();
    }
    else if (bridgeType == "yarp2ros")
    {
        // Generating .msg file
        generateRosMessage(groups);
        // Compiling the ros nodes if there was an update
        if (updated)
        {
            compileRosNodes(lastActualEdit,dataType);
        }

        // Launching the thread
        //....................................................
        switch (dataType)
        {
            case 0:
                thread.reset(new DataThread(name, yarpPort, rosTopic, groups, rate, fd)); // We're sending classic data
                break;
            case 1:
                thread.reset(new ImageThread(name, yarpPort, rosTopic, rate, fd)); // We're sending an image
                break;
            default:
                cerr << "The .ini file does not properly gives the data type (0 = data, 1 = image)" << endl;
                close();
                return false;
        }
        if ( !thread->start() )
        {
            cout << "[yarp]::Error : the thread could not start" << endl;
            thread.reset();
            close();
            return false;
        }
    }
    else
    {
        cout << "The type of the bridge can only be yarp2ros or ros2yarp..." << endl;
        close();
        return false;
    }
    return true;
}

bool ConvertModule::close()
{
    thread->stop();
    return true;
}

double ConvertModule::getPeriod()
{ 
    return 1.0;  
}

bool ConvertModule::updateModule() 
{
    if(count%60==0)
        cout << " YARP ROS BRIDGE GENERATOR alive since " << (count/60) << " mins ... " << endl;
    count++;
    return true; 
}

bool ConvertModule::interruptModule()
{
    return true;
}

bool ConvertModule::checkPort(string devicePort, int dataType)
{
    char type;
    if (!dataType && devicePort.find("cam") != string::npos)
    {
        cout << "You are not sending raw data and yet the device port you selected contains \"cam\"." << endl;
        cout << "This may be intentional, but if it is not, please modify your .ini file" << endl;
        do
        {
            cout << "Would you like to continue anyway ? [y/n]" << endl;
            cin >> type;
        } while ( !cin.fail() && type!='y' && type!='n' );
        if ( type == 'n' )
        {
            cout << "Terminating the program..." << endl;
            close();
            return false;
        }
    }
    return true;
}

void ConvertModule::compileRosNodes(string lastActualEdit, int dataType)
{
    char* sPath = ::getenv("SHELL");
    switch (fork())
    {   
        case -1:
            cerr << "Fork error" << endl;
            close();
        case 0:
            if (dataType)
                execlp(sPath,sPath,"compile_ros.sh", lastActualEdit.c_str(), "image", NULL);
            execlp(sPath,sPath,"compile_ros.sh", lastActualEdit.c_str(), NULL);
            break;
        default:
            wait(NULL);
            break;
    }
}

bool ConvertModule::checkUpdate(yarp::os::ResourceFinder &rf, string &lastActualEdit)
{
    struct stat iniStat;
    string lastRegisteredEdit;
    string path(rf.findFile("from").c_str());

    if (stat(path.c_str(), &iniStat) == -1)
    {
        cerr << "stat() error when trying to determine the latest edit of the .ini file" << endl;
    }
    lastActualEdit = ctime(&iniStat.st_mtime);
    lastActualEdit.erase(lastActualEdit.find('\n',0),1); 

    fstream lastEditFile;
    lastEditFile.open("lastedit.txt",fstream::in);
    if (lastEditFile.is_open())
    {
        getline(lastEditFile,lastRegisteredEdit);
        lastEditFile.close();
    }

    return lastRegisteredEdit.compare(lastActualEdit);
}

bool ConvertModule::tryAutomaticMode()
{
    groups.clear();
    
    int size = 0;
    string str;
    if (bridgeType == "yarp2ros")
    {
        ConstString pname = string(yarpPort).c_str();
        Port p;
        if (!p.open("..."))
        {
            cerr << "Unable to create temporary port to use automatic mode" << endl;
            close();
            return false;
        }
        if (!Network::connect(pname,p.getName()))
        {
            cerr << "Unable to connect to the temporary port used for automaic mode" << endl;
            close();
            return false;
        }
        Bottle b;
        p.read(b);
        getTypeString(b,"v",size,str))
        p.interrupt();
        p.close();
    }
    else
    {
        try // always check for Python_exceptions
        {
            vector<string> args;
            args.push_back(rosTopic);
            str = pythonWrap("automaticrosprobing","getString",args);
            stringstream in(str);
            size = std::distance(istream_iterator<string>(in), istream_iterator<string>()) / 2;
        }
        catch (Python_exception ex)
        {
            cerr << ex.what();
            close();
            return false;
        }
    }
    numberOfGroups = size;
    stringstream in(str);
    for (int i = 0; i < size; i++)
    {
        string second,first;
        
        in  >> second;
        in >> first;
        pair<string,string> header = make_pair(first,second);

        groups.push_back(header);
    }
    return true;
}

string ConvertModule::addPart(string type, string name, int size) 
{
    ostringstream buf;
    string result;
    for (int i = 0; i < size; i++)
    {
        if (i < 10)
            result.append(type+" "+name+"_"+to_string(i)+" "); 
    }
    buf << result;
    return buf.str();
 }

void ConvertModule::getTypeString(Bottle& b, string root, int &size, string &str) 
{
    ostringstream oss;
    int code = b.getSpecialization();
    
    bool specialized = (code>0);
    if (code == BOTTLE_TAG_INT) {
        size = b.size();
        oss << addPart("int32",root,b.size()) << endl;
        str = oss.str();
        return true;
    }
    if (code == BOTTLE_TAG_DOUBLE) {
        size = b.size();
        oss << addPart("float64",root,b.size()) << endl;
        str = oss.str();
        return true;
    }
    for ( int i = 0; i< b.size(); i++) {

        Value& v = b.get(i);
        ostringstream val_name;
        
        val_name << root << i;
        
         if (v.isBlob()) 
        {
            size += v.asBlobLength();
            oss << addPart("int8",val_name.str(),v.asBlobLength()) << " ";
        } 
        else 
        {
            cerr << "The automatic mode does not yet support what this port is sending." << endl;
            cerr << "Automatic mode works best with simple vectors (floats, ints...)." << endl;
            close();
            return false;
        } 
    }
    str = oss.str();
    return true;
 }