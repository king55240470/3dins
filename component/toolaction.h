#ifndef TOOLACTION_H
#define TOOLACTION_H
#include<QAction>
#include<QString>
#include<QWidget>
enum ToolActionKind{
    SaveAction,
    ConstructAction,
    CoordAction,
    FindAction,
    ViewAngleAction
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

#endif // TOOLACTION_H
