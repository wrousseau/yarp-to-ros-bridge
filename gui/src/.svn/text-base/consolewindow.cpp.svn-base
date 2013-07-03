#include "include/consolewindow.h"
#include "ui_consolewindow.h"

ConsoleWindow::ConsoleWindow(QWidget *parent) :
    QTextEdit(parent),
    ui(new Ui::ConsoleWindow)
{
    ui->setupUi(this);
}

ConsoleWindow::~ConsoleWindow()
{
    delete ui;
}

void ConsoleWindow::closeEvent(QCloseEvent *event)
{
    emit aboutToClose();
    QTextEdit::closeEvent(event);
}

void ConsoleWindow::setProcessPid(Q_PID pid)
{
    processPid = pid;
}
