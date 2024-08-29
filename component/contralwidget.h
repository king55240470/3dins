

#ifndef CONTRALWIDGET_H
#define CONTRALWIDGET_H
#include <QApplication>
#include <QMainWindow>
#include <QPushButton>
#include <QVBoxLayout>
#include <QDialog>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include<QSettings>
#include"toolwidget.h"
class ContralWidget : public QDialog {
public:
    ContralWidget(ToolWidget* toolWidget_,QWidget *parent = nullptr);
    void saveCheckBoxState();
    void loadCheckBoxState();
protected:
    void closeEvent(QCloseEvent* event)override;
    bool  ActionIsChecked[4][100];
    QString ActionName[10][10];
    QString iconNames_First[59];
    QString iconNames_Second[88];
    QString iconNames_Third[6];
    QTreeWidget *treeWidget;
    ToolWidget* toolWidget;
};



#endif // CONTRALWIDGET_H


