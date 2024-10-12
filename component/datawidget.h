#ifndef DATAWIDGET_H
#define DATAWIDGET_H

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QComboBox>
#include <QTableWidget>
#include <QString>
#include "mainwindow.h"
#include "geometry/centitytypes.h"
class DataWidget : public QWidget
{
    Q_OBJECT
public:
    explicit DataWidget(QWidget *parent = nullptr);
    void updateinfo();
    void getobjindex(int objindex);
    void getentityindex(int entityindex);
    void OnComboBoxIndexChanged(int);
private:
    QVBoxLayout *vlayout;
    QHBoxLayout *hlayout;
    QLabel *label1;
    QLabel *label2;
    QLabel *label3;
    QComboBox *box;
    QTableWidget *table;
    MainWindow *m_pMainWin=nullptr;
    int index=0;
    bool tempIn=false; // 标志临时坐标系是否在box
};

#endif // DATAWIDGET_H
