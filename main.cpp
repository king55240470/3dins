#include "mainwindow.h"
// #include "mydll.h"


#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    QSplashScreen splash;
    splash.setPixmap(QPixmap(":/style/logo.png").scaled(400, 400));
    splash.show();
    MainWindow w;
    w.setWindowState(Qt::WindowMaximized);//使窗口最大化显示
    w.setWindowIcon(QIcon(":/style/ruler.png"));
    w.setWindowTitle("三维工业测量软件");
    splash.finish(&w);
    w.show();

    // MyDll lib;
    return a.exec();
}
