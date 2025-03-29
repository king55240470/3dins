#ifndef WORKER_H
#define WORKER_H

#include <QWidget>
#include<QDebug>
#include<QThread>
//#include "mainwindow.h"
//#include"component/toolwidget.h"
class Worker : public QWidget
{
    Q_OBJECT
public:
    explicit Worker(QWidget  *parent = nullptr);
    //MainWindow *m_pMainWin=nullptr;
public slots:
    void doWork() {
        // 通过信号触发主线程操作
        emit requestSaveOperation(0); // 0=SaveTxt
        QThread::msleep(700); // 模拟耗时操作
        emit progress(25);

        emit requestSaveOperation(1); // 1=SaveWord
        QThread::msleep(700);
        emit progress(50);

        emit requestSaveOperation(2); // 2=SaveExcel
        QThread::msleep(700);
        emit progress(75);

        emit requestSaveOperation(3); // 3=SavePdf
        QThread::msleep(700);
        emit progress(100);

        emit finished();
    }

signals:
    void progress(int value);
    void finished();
    void requestSaveOperation(int type); // 新增信号通知主线程执行保存
};

#endif // WORKER_H
