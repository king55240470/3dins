

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
    bool  ActionIsChecked[ToolWidget::ToolBarCount][100];
    QString ActionName[ToolWidget::ToolBarCount][100];
    QString* iconNames_First;
    QString* iconNames_Second;
    QString* iconNames_Third;
    QTreeWidget *treeWidget;
    ToolWidget* toolWidget;
    QTreeWidgetItem** fatherItems;
    ~ContralWidget();
};



#endif // CONTRALWIDGET_H


