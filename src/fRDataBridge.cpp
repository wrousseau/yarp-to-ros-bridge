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
 * \file fRDataBridge.cpp
 * \brief Sending data from a ROS topic to a YARP port
 * \author Woody Rousseau
 * \version 0.1
 * \date 05/06/13
 *
 * Uses a Python wrapper
 */


#include "fRDataBridge.h"

FRDataBridge::FRDataBridge(int* _fd, string _topicName, string _yarpPort, field _rosGroups) : 
FRBridge(_fd, _topicName, _yarpPort), rosGroups(_rosGroups)
{
}

bool FRDataBridge::writeData()
{
    try // always check for Python_exceptions
    {
        vector<string> args;
        args.push_back(to_string(fd[P_WRITE]));
        args.push_back(topicName);
        pythonWrap("stringifymsg","stringify",args);         
        return true;
    }
    catch (Python_exception ex)
    {
        cerr << ex.what();
    }
    return true;
}

bool FRDataBridge::readRosData()
{
    FILE* fromRosPipe = NULL;
    if ( (fromRosPipe = fdopen(fd[P_READ],"r")) == NULL )
    {
        cerr << "Fdopen" << endl;
        return false;
    }

    BufferedPort<Bottle> port;
    port.open(yarpPort.c_str());

    char receivedData[1000];
    vector<int> types;
    for (const auto& group : rosGroups)
    {
        types.push_back(getYarpTypeInt(group.second));
    }

    while (true)
    {
        Bottle& output = port.prepare();
        output.clear();
        unsigned size;

        while (read(fd[P_READ],receivedData,1000) <= 0)
        {}

        vector<pair<string,unsigned>> data = processIncomingData(receivedData);

        auto dit = data.cbegin();
        for (int i = 0; i!=types.size() && dit!=data.cend(); ++i)
        {
            size = dit->second;
            for (unsigned i = 0; i < size; i++)
            {
                switch (types[i])
                {
                    case STRING_T:
                        output.addString(dit->first.c_str());
                        break;
                    case DOUBLE_T:
                        output.addDouble(atof(dit->first.c_str()));
                        break;
                    case INT_T:
                        output.addInt(atoi(dit->first.c_str()));
                        break;
                    case BOOL_T:
                        output.addInt(boolStringToInt(dit->first.c_str()));
                        break;
                    default:
                        break;
                }
                ++dit;
            }
        }
        port.write();
    }
}

bool FRDataBridge::boolStringToInt(string str) 
{
    if (str.at(0) == '1')
        return true;
    if (str.at(0) == '0')
        return false;
    std::transform(str.begin(), str.end(), str.begin(), ::tolower);
    istringstream is(str);
    bool b;
    is >> boolalpha >> b;
    return b;
}

int FRDataBridge::getYarpTypeInt(string type)
{
    if ( auto pos = type.find("[") != string::npos)
    {
        type = type.substr(0,type.size()-pos-1);
    }
    vector<string> intTypes = {"int","uint","unsigned","unsigned int","short","char","unsigned char","short","short int", "long", "long int", "unsigned long", "unsigned long int", "long long", "long long int", "unsigned short", "unsigned short int","int8","uint8","int16","uint16","int32","uint32","int64","uint64","byte"};
    vector<string> doubleTypes = {"float32","float64","double","long double"};
    vector<string> stringTypes = {"string"};

    if (std::find(stringTypes.begin(), stringTypes.end(), type) != stringTypes.end())
    {
        return STRING_T;
    }
    if (std::find(doubleTypes.begin(), doubleTypes.end(), type) != doubleTypes.end())
    {
        return DOUBLE_T;
    }
    if (std::find(intTypes.begin(), intTypes.end(), type) != intTypes.end())
    {
        return INT_T;
    }
    if (type.find("bool") != std::string::npos)
    {
        return BOOL_T;
    }

    cerr << "Type " << type << " unidentified..." << endl;
    return -1;
}

vector<pair<string,unsigned>> FRDataBridge::processIncomingData(const string &receivedData)
{
    vector<pair<string,unsigned>> data;
    unsigned size = 1;
    istringstream iss(receivedData);
    istream_iterator<string> in_iter(iss);
    istream_iterator<string> eof;
    while (in_iter != eof)
    {

        string tmp;
        if (in_iter->find("\"\"") != string::npos)
        {
            tmp.append(in_iter->substr(2));
            in_iter++;
            while (in_iter->find("\"\"") == string::npos)
            {
                tmp.append(" ");
                tmp.append(*in_iter);
                in_iter++;
            }
            tmp.append(" ");
            tmp.append(in_iter->substr(0,in_iter->size()-2));
            in_iter++;
        }
        else if (in_iter->find("\"v\"") != string::npos)
        {
            size = atoi(in_iter->substr(3).c_str());
            in_iter++;
            while (in_iter->find("\"v\"") == string::npos)
            {
                data.push_back({*in_iter,size});
                in_iter++;
            }
            data.push_back({in_iter->substr(0,in_iter->size()-3),size});
            in_iter++;
            continue;
        }
        else
        {
            tmp = *in_iter++;
        }
        data.push_back({tmp,size});
    }
    return data;
}
