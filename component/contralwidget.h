
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
#include <QMouseEvent>


#include"toolwidget.h"
#include"toolaction.h"
class ContralWidget : public QDialog {
     Q_OBJECT
public:

    ContralWidget(ToolWidget* toolWidget_,QWidget *parent = nullptr);
    void saveCheckBoxState();
    void loadCheckBoxState();
      ~ContralWidget();
    void addActionItems(int actionIndex, int actionNum, const QStringList *actionNameList,const QStringList *actionIconPathList) ;
    void mousePressEvent(QMouseEvent *event) override;
protected:
    QList<UniqueToolBar*> UniqueToolbarList;

    const int save_action_index_=2;
    const int find_action_index_=0;
    const int construct_action_index_=1;
    const int coord_action_index_=3;
    const int view_angle_action_index_=4;

    void createTreeWidgetItem();
    void closeEvent(QCloseEvent* event)override;


    bool action_is_checked_[ToolWidget::ActionKindNum][12];


    const QStringList* save_action_name_list_;
    const QStringList* find_action_name_list_;
    const QStringList* construct_action_name_list_;
    const QStringList* coord_action_name_list_;
    const QStringList * view_angle_action_name_list_;

    const QStringList* save_action_iconpath_list_;
    const QStringList* find_action_iconpath_list_;
    const QStringList* construct_action_iconpath_list_;
    const QStringList* coord_action_iconpath_list_;
    const QStringList * view_angle_action_iconpath_list_;



    QTreeWidget *treeWidget;

    ToolWidget* toolWidget;

    QTreeWidgetItem** fatherItems;


private slots:
    void onItemSelectionChanged();


};



#endif // CONTRALWIDGET_H


