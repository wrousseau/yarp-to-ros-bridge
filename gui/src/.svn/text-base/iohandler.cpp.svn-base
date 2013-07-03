#include "include/iohandler.h"


IOHandler::IOHandler(std::map<std::string,std::string> &_data, std::map<std::string,std::string> &_groups, const QString &name) :
    QFile(name), data(_data), groups(_groups) {}

void IOHandler::saveIni()
{
    if (!open(QIODevice::WriteOnly | QIODevice::Text))
        return;
    QTextStream out(this);
    int numberofGroups = atoi(data.find("numberofgroups")->second.c_str());
    out << "[general]" << endl;
    out << "# Name of the module you are trying to use thanks to the bridge" << endl;
    out << "name " << QString(data.find("name")->second.c_str()) << endl;
    out << "# yarp2ros = We are sending data from yarp to ros" << endl;
    out << "# ros2yarp = We are sending data from ros to yarp" << endl;
    out << "bridgetype " << getBridgeType(data.find("bridgetype")->second) << endl;
    out << "# 0 = normal data, 1 = image" << endl;
    out << "# If it is an image, most fields and groups will not be processed by the thread" << endl;
    out << "# But you can leave them there anyway, they will not be processed" << endl;
    out << "datatype " << QString(data.find("datatype")->second.c_str()) << endl;

    out << "# How many individual data are coming/going from the port/topic ?" << endl;
    out << "# Example : If you are sending position data, that is 3 doubles, so write :" << endl;
    out << "# numberofgroups 6" << endl;
    if (numberofGroups != 0)
        out << "numberofyarpgroups " << numberofGroups << endl;
    out << "# Give some names to each individual data. Mandatory and if possible, relevant" << endl;
    out << "# Example :" << endl;
    out << "# groups x y z vx vy vz" << endl;
    if(numberofGroups != 0)
        out << "groups ";
    auto iter = groups.cbegin();
    while ( iter != groups.cend() )
    {
        if (iter != groups.cbegin())
            out << " ";
        out << QString(iter->second.c_str());
        ++iter;
    }
    out << endl;

    out << "# Rate at which the thread is running" << endl;
    out << "rate 10" << endl;
    out << endl;

    out << "# Would you like the bridge to probe the port/topic to figure out the types ?" << endl;
    out << "# 0 to disable, 1 to enable automatic mode" << endl;
    out << "automatic " << QString((data.find("automatic")->second).c_str()) << endl;

    out << "[yarp]" << endl;
    out << "#  The yarp port (output or input). If you run multiple bridges, make sure this is not repeated." << endl;
    out << "port " << QString((data.find("port")->second).c_str()) << endl;
    out << "[ros]" << endl;
    out << "# The topic (output or input). If you run multiple bridges, make sure this is not repeated." << endl;
    out << "topic " << QString((data.find("topic")->second).c_str()) << endl;

    out << "# Enter here each individual data in that format (without the # symbols)" << endl;
    out << "# [data_#namegiven#]" << endl;
    out << "# type #type#" << endl;
    out << "# Example for a doubled called x (which was present in the groups list)" << endl;
    out << "# [data_x]" << endl;
    out << "# type double" << endl;
    iter = groups.cbegin();
    while ( iter != groups.cend() )
    {
        out << "[data_" << QString(iter->second.c_str()) << "]" << endl;
        out << "type" << QString(iter->first.c_str()) << endl;
        ++iter;
    }
    this->close();

}

QString IOHandler::getBridgeType(std::string type)
{
    if (type == "0")
        return "yarp2ros";
    else
        return "ros2yarp";
}

QString IOHandler::getField(QString field)
{
    if (!open(QIODevice::ReadOnly | QIODevice::Text))
        return "";
    QTextStream in(this);
    QString line;
    while ( ( !( line = in.readLine() ).isNull() ) && !line.startsWith(field) )
    {;}
    if (line.isNull())
        return "";
    close();
    return line.right(line.size()-field.size()-1);

}

QString IOHandler::getGroup(QString type, int i)
{
    QString fields = getField(type);
    QStringList list = fields.split(" ");
    return list.at(i);
}

int IOHandler::checkComboBox(QString field)
{
    if (!open(QIODevice::ReadOnly | QIODevice::Text))
        return -1;
    QTextStream in(this);
    QString line;
    while ( ( !( line = in.readLine() ).isNull() ) && !line.startsWith(field) )
    {line.clear();}
    if (line.isNull())
        return -1;
    close();
    QString result = line.right(line.size()-field.size()-1);
    if (result == "yarp2ros")
        return 0;
    if (result == "ros2yarp")
        return 1;
    return result.toInt();
}

QString IOHandler::getType(QString name)
{
    if (!open(QIODevice::ReadOnly | QIODevice::Text))
        return "";
    QTextStream in(this);
    QString line;
    QString field = "[data_";
    field.append(name+"]");
    while ( ( !( line = in.readLine() ).isNull() ) && !line.startsWith(field) )
    {;}
    line = in.readLine();
    if (line.isNull())
        return "";
    close();
    return line.right(line.size()-5);

}

