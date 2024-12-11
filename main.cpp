#include "mainwindow.h"
// #include "mydll.h"


#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.setWindowState(Qt::WindowMaximized);//使窗口最大化显示
    w.show();

    // auto styleFile = QFile(":/style/MyQss_2.qss");
    // styleFile.open(QFile::ReadOnly);
    // if(styleFile.isOpen()){
    //     QString styleSheets = QLatin1String(styleFile.readAll());
    //     a.setStyleSheet(styleSheets);
    //     styleFile.close();
    // }
    // else {
    //     qDebug() << "打开样式文件失败";
    // }

    // MyDll lib;
    return a.exec();
}
