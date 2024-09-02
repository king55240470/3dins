
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
     Q_OBJECT
public:

    ContralWidget(ToolWidget* toolWidget_,QWidget *parent = nullptr);
    void saveCheckBoxState();
    void loadCheckBoxState();
      ~ContralWidget();

protected:

    const int save_action_index_=2;
    const int find_action_index_=0;
    const int construct_action_index_=1;
    const int coord_action_index_=3;

    void createTreeWidgetItem();
    void closeEvent(QCloseEvent* event)override;

    bool action_is_checked_[ToolWidget::ActionKindNum][12];

    QString* iconNames_First;
    QString* iconNames_Second;
    QString* iconNames_Third;

    const QStringList* save_action_name_list_;
    const QStringList* find_action_name_list_;
    const QStringList* construct_action_name_list_;
    const QStringList* coord_action_name_list_;

    QTreeWidget *treeWidget;

    ToolWidget* toolWidget;

    QTreeWidgetItem** fatherItems;


};



#endif // CONTRALWIDGET_H


