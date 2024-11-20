#ifndef TOOLACTION_H
#define TOOLACTION_H
#include<QAction>
#include<QString>
#include<QWidget>
#include<QToolBar>
enum ToolActionKind{
    SaveAction,
    ConstructAction,
    CoordAction,
    FindAction,
    ViewAngleAction
};
class ToolBarGathter;
class ToolAction;
class UniqueToolBar;

class UniqueToolBar{
public:
    QStringList m_nameList;
    QStringList m_pathList;
    QList<QAction*> m_actionList;
    QList<QAction*> m_chosenActionList;
    QStringList m_chosenNameList;
private:
    int m_num;
    QString m_name;
    ToolActionKind m_actionKind;
    QWidget* m_parent;
public: 
    //构造和析构函数
    UniqueToolBar();
    ~UniqueToolBar();
    void setChosenNameList(QStringList);

    void setParent(QWidget* parent){
        m_parent=parent;
    }
    QWidget* getParent(){
        return m_parent;
    }
    void clear();
    bool loadAction(QWidget* parent=nullptr);
    QStringList& getNameList(){
        return m_nameList;
    }
    QStringList& getPathList(){
        return m_pathList;
    }
    QList<QAction*>& getActionList(){
        return m_actionList;
    }
    int getNum(){
        return m_num;
    };
    void setNum(int num){
        m_num=num;
    }
    QString getName(){
        return m_name;
    }
    void setName(QString name){
        m_name=name;
    }
    ToolActionKind getactionKind(){
        return m_actionKind;
    };
    void setActionKind(ToolActionKind actionKind){
        m_actionKind=actionKind;
    }

};



class ToolAction :public QAction
{
    Q_OBJECT
public:
    ToolAction(QWidget* parent=nullptr);
    void setToolActionKind(ToolActionKind);
    void setName(QString ) ;
private:
    ToolActionKind action_kind_;
    QString name_;
};



class ToolBarGathter{
private:
    QList<UniqueToolBar*> m_UniqueToolBarList;
    int m_allActionNum;
    int m_nToolbarNum;
    int m_singalToolBarActionNum;
    QList<QToolBar*>  toolBars;
public:

    ToolBarGathter();
    ~ToolBarGathter();
    void setSingalToolBarActionNum(int num){
        m_singalToolBarActionNum=num;
    }
    int getSingalToolBarActionNum(int num){
        return m_singalToolBarActionNum;
    }
    void addToWidget(QWidget* toolWidget);
    void addUniqueToolBar(UniqueToolBar* toolbar){
        m_UniqueToolBarList.push_back(toolbar);
    }
    void clear();
    QList<UniqueToolBar*> getUniqueToolBarList(){
        return m_UniqueToolBarList;
    }
};



#endif // TOOLACTION_H
