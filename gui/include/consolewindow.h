#ifndef CONSOLEWINDOW_H
#define CONSOLEWINDOW_H

#include <signal.h>
#include <sys/types.h>
#include <QTextEdit>
#include <QDebug>
#include <QProcess>

namespace Ui {
class ConsoleWindow;
}

class ConsoleWindow : public QTextEdit
{
    Q_OBJECT
    
public:
    explicit ConsoleWindow(QWidget *parent = 0);
    ~ConsoleWindow();
    virtual void closeEvent ( QCloseEvent * event );
    void setProcessPid(Q_PID pid);
    
private:
    Ui::ConsoleWindow *ui;
    Q_PID processPid;

signals:
    void aboutToClose();
};

#endif // CONSOLEWINDOW_H
