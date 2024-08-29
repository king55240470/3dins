
#ifndef TOOLWIDGET_H
#define TOOLWIDGET_H
#include <QWidget>
#include <QToolBar>
#include <QAction>
#include <QVBoxLayout>
#include"toolaction.h"
#define EACH_TOOLACTION_NUM 50;
const unsigned int eachToolActionNum=50;
class ToolWidget : public QWidget {
    Q_OBJECT

public:
    explicit ToolWidget(QWidget *parent = nullptr);
    ~ToolWidget();
    void deleteToolAction(ToolActionKind,QString);
    void addToolAction(ToolActionKind,QString);
    void deleteToolActions(ToolActionKind);
    void addToolActions(ToolActionKind);
    void recreateToolBars(ToolActionKind* ,int );
    void clear();
    QString iconNames_First[59];
    QString iconNames_Second[88];
    QString iconNames_Third[6];
   static const int FirstToolBarActions_Num=59;
    static const int SecondToolBarActions_Num=88;
   static const int ThirdToolBarActions_Num=6;
    static const int SingalToolBarActionNum=12;
   static const int ActionSize_Length=20;
    static const int ActionSize_Width=20;

private:
    void createToolBars();
    ToolAction** FirstToolBarActions;
    ToolAction** SecondToolBarActions;
    ToolAction** ThirdToolBarActions;
    ToolAction** FourthToolBarActions;
    ToolAction* cubeActions[eachToolActionNum];
    ToolAction * cuboidActions[eachToolActionNum];
     ToolAction * sphereActions[eachToolActionNum];
     ToolAction * coneActions[eachToolActionNum];
     QToolBar ** toolBars;

};
#endif // TOOLWIDGET_H
