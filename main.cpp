#include "mainwindow.h"

#include "mydll.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.setWindowState(Qt::WindowMaximized);//使窗口最大化显示
    w.show();
    MyDll lib;
    qDebug()<<lib.add(3,3);
    return a.exec();
}
