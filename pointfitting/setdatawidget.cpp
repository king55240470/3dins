#include "setdatawidget.h"
#include "mainwindow.h"
#include "pointfitting/fittingplane.h"
class VtkWidget;

#include <QMessageBox>

setDataWidget::setDataWidget(QWidget *parent)
    : QWidget{parent}
{
    m_pMainWin=(MainWindow*)parent;
    p_cloudptr.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    fittingPlane.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    plane=nullptr;
}

void setDataWidget::setPlaneData(pcl::PointXYZRGB searchPoint,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudptr){
    p_point=searchPoint;
    p_cloudptr=cloudptr;
    p_dialog = new QDialog(this);
    p_dialog->resize(400,150);
    p_dialog->setWindowTitle("设置邻域和阈值");
    p_layout = new QGridLayout(p_dialog);
    p_lab1 = new QLabel("请输入邻域（一般设置为0.1）：");
    p_lab2 = new QLabel("请输入距离阈值（一般设置为0.01）：");
    p_rad = new QLineEdit();
    p_rad->setText("0.1");
    p_dis = new QLineEdit();
    p_dis->setText("0.01");
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
    p_dialog->close();
    if(p_dis->text().toDouble()<=0||p_rad->text().toDouble()<=0){
        QMessageBox *messagebox=new QMessageBox();
        messagebox->setText("输入大于0的数");
        messagebox->setIcon(QMessageBox::Warning);
        messagebox->show();
        messagebox->exec();
        return;
    }
    plane=new FittingPlane();
    plane->setRadious(p_rad->text().toDouble());  // 设置半径
    plane->setDistance(p_dis->text().toDouble());  // 设置距离阈值
    auto planeCloud= plane->RANSAC(p_point,p_cloudptr);
    qDebug()<<planeCloud->size();
    pcl::copyPointCloud(*planeCloud, *fittingPlane);
    if(planeCloud==nullptr){
        qDebug()<<"出现了点云空指针";
    }

    // m_pMainWin->getPointCloudListMgr()->getProductCloudList().push_back(*plane->RANSAC(p_point, p_cloudptr).get());
}

FittingPlane *setDataWidget::getPlane(){
    return plane;
}
