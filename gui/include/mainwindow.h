#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <cstdlib>
#include <cstdio>
#include <string>
#include <map>

#include <QMainWindow>
#include <QMenu>
#include <QMenuBar>
#include <QLabel>
#include <QTextEdit>
#include <QMessageBox>
#include <QString>
#include <QDebug>
#include <QFileDialog>
#include <QProcess>
#include <QKeyEvent>
#include <include/consolewindow.h>


#include "include/iohandler.h"
#include "include/consolewindow.h"

using namespace std;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    void createActions();
    void createMenus();
    bool isPartiallyFilled();
    bool checkFields();
    void clearAllFields();
    inline const QString boolToNumberQString(bool b);
    bool checkPort(int dataType, std::string devicePort);







private slots:
    void addData();
    void removeData();
    void copy();
    void cut();
    void paste();
    void displayMessageBox();
    bool save(bool tmp = false);
    void trigger(int i);
    void open();
    void newConfig();
    void launch();
    void readyReadStandardOutput();
    void readyReadStandardError();
    void terminateLaunch();
    void iniEdited();









private:
    Ui::MainWindow *ui;
    QMenu *fileMenu;
    QMenu *editMenu;
    QMenu *helpMenu;
    QAction *newAct;
    QAction *openAct;
    QAction *saveAct;
    QAction *saveAsAct;
    QAction *exitAct;
    QAction *cutAct;
    QAction *copyAct;
    QAction *pasteAct;
    QAction *aboutAct;
    QLabel *infoLabel;

    QString currentFileName;

    QMessageBox *aboutBox;

    QProcess* process;

    ConsoleWindow *contentsWindow;

    bool sessionLaunched;
    bool sameTmp;

};

#endif // MAINWINDOW_H
