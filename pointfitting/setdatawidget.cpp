#include "setdatawidget.h"
#include "mainwindow.h"
#include "pointfitting/fittingplane.h"

setDataWidget::setDataWidget(QWidget *parent)
    : QWidget{parent}
{
    m_pMainWin=(MainWindow*)parent;
}

void setDataWidget::setPlaneData(pcl::PointXYZRGB searchPoint,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudptr){
    p_point=searchPoint;
    p_cloudptr=cloudptr;
    p_dialog = new QDialog(this);
    p_dialog->resize(400,150);
    p_layout = new QGridLayout(p_dialog);
    p_lab1 = new QLabel("请输入邻域：");
    p_lab2 = new QLabel("请输入距离阈值：");
    p_rad = new QLineEdit();
    p_dis = new QLineEdit();
    p_btn = new QPushButton("确定");
    p_layout->addWidget(p_lab1,0,0,1,1);
    p_layout->addWidget(p_rad,0,1,1,2);
    p_layout->addWidget(p_lab2,1,0,1,1);
    p_layout->addWidget(p_dis,1,1,1,2);
    p_layout->addWidget(p_btn,2,2,1,1);
    p_dialog->setLayout(p_layout);
    p_dialog->show();
    connect(p_btn,&QPushButton::clicked,this,&setDataWidget::PlaneBtnClick);
    p_dialog->exec();
}

void setDataWidget::PlaneBtnClick(){
    FittingPlane *plane=new FittingPlane();
    plane->setRadious(p_rad->text().toDouble());  // 设置半径
<<<<<<< HEAD
    plane->setDistance(p_dis->text().toDouble());  // 设置距离阈值
    plane->RANSAC(p_point, p_cloudptr);
=======
    plane->setDistance(p_dis->text().toInt());  // 设置距离阈值
    *fittingPlane=plane->RANSAC(p_point, p_cloudptr);
>>>>>>> 80ae09fa06a8b9e3451c52defcb08a33a8ba821e
    //p_dialog->close();
}
