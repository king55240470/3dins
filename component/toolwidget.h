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
    void deleteToolActions(ToolActionKind);
    int  addToolActions(ToolActionKind ,QString * ,int ,int =-1);//返回添加后最后toolBar的索引
    void clear();
    QString * geticonNames_First() const;
    QString * geticonNames_Second()const;
    QString * geticonNames_Third()const;
   static const int FirstToolBarActions_Num=59;
    static const int SecondToolBarActions_Num=88;
   static const int ThirdToolBarActions_Num=6;
    static const int SingalToolBarActionNum=12;
   static const int ActionSize_Length=20;
    static const int ActionSize_Width=20;
  static const int FirstToolBarIndex=0;
  static  const int SecondToolBarIndex=1;
  static const int ThirdToolBarIndex=2;
  static const int ToolBarCount=3;
   int nToolbarNum() const;
   void setNToolbarNum(int newNToolbarNum);

 private:
    void createToolBars();//内部函数,在构造时使用
    ToolAction** FirstToolBarActions;
    ToolAction** SecondToolBarActions;
    ToolAction** ThirdToolBarActions;
    ToolAction** FourthToolBarActions;
    QToolBar ** toolBars;
    QString* iconNames_First;
    QString* iconNames_Second;
    QString* iconNames_Third;
    int m_nToolbarNum;

};
#endif // TOOLWIDGET_H
