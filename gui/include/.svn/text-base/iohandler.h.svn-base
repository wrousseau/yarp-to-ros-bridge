#ifndef IOHANDLER_H
#define IOHANDLER_H

#include <QFile>
#include <QTextStream>
#include <QStringList>
#include <QDebug>

class IOHandler : public QFile
{

public:
    IOHandler(std::map<std::string,std::string> &data, std::map<std::string,std::string> &groups, const QString &name);
    void saveIni();
    QString getBridgeType(std::string type);
    QString getField(QString field);
    int checkComboBox(QString field);
    QString getGroup(QString type, int i);
    QString getType(QString name);


    
signals:
    
public slots:

private:
    std::map<std::string,std::string> data;
    std::map<std::string,std::string> groups;
    
};

#endif // IOHANDLER_H
