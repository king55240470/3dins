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
    void updateele(int i);
    // void UpdateInfo();
private:
    QVBoxLayout *vlayout;
    QHBoxLayout *hlayout;
    QLabel *label1;
    QLabel *label2;
    QLabel *label3;
    QComboBox *box;
    QTableWidget *table;
    MainWindow *m_pMainWin=nullptr;

};

#endif // DATAWIDGET_H
