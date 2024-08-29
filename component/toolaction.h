#ifndef TOOLACTION_H
#define TOOLACTION_H
#include<QAction>
#include<QString>
#include<QWidget>
enum ToolActionKind{
    Cube,//立方体
    Cuboid,//长方体
    Sphere,//球体
    Cylinder,//圆柱体
    Cone,//圆锥
    Pyramid,//金字塔
    Prism,//棱柱体
    Ellipsoid,//椭球体
    Hyperboloid,//双曲面
    Conic ,//锥形体
    FirstToolBarAction,
    SecondToolBarAction,
    ThirdToolBarAction,
    FourthToolBarAction

};
class ToolAction :public QAction
{
public:
    ToolAction(ToolActionKind ActionKind ,QWidget* parent=nullptr,QString iconPath="",int AddLine=0,QString Name="");
    ToolActionKind actionKind;
    int addLine;
    QString name;
};

#endif // TOOLACTION_H
