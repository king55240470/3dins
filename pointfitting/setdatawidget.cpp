#include "setdatawidget.h"
#include "mainwindow.h"
#include "pointfitting/fittingplane.h"
#include "pointfitting/fittingCylinder.h"
#include "pointfitting/fittingsphere.h"
#include "pointfitting/fittingcone.h"
#include "pointfitting/fittingline.h"

class VtkWidget;

#include <QMessageBox>

setDataWidget::setDataWidget(QWidget *parent)
    : QWidget{parent}
{
    m_pMainWin=(MainWindow*)parent;

    cloudptr.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

    fittingPlane.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    planeCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    plane=nullptr;

    fittingCylinder.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    cylinderCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    cylinder=nullptr;

    fittingSphere.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    sphereCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    sphere=nullptr;

    fittingCone.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    coneCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    cone=nullptr;

    fittingLine.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    lineCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    line=nullptr;

    rad = new QLineEdit();
    dis = new QLineEdit();

    rad->setText(QString::number(0.1));
    dis->setText(QString::number(0.01));

}

//平面
void setDataWidget::setPlaneData(pcl::PointXYZRGB searchPoint,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
    point=searchPoint;
    cloudptr=cloud;
    dialog = new QDialog(this);
    dialog->resize(400,150);
    dialog->setWindowTitle("设置邻域和阈值");
    layout = new QGridLayout(dialog);
    lab1 = new QLabel("请输入邻域（一般设置为0.1）：");
    lab2 = new QLabel("请输入距离阈值（一般设置为0.01）：");
    // rad = new QLineEdit();
    // rad->setText("0.1");
    // dis = new QLineEdit();
    // dis->setText("0.01");
    btn = new QPushButton("确定");
    layout->addWidget(lab1,0,0,1,1);
    layout->addWidget(rad,0,1,1,2);
    layout->addWidget(lab2,1,0,1,1);
    layout->addWidget(dis,1,1,1,2);
    layout->addWidget(btn,2,2,1,1);
    dialog->setLayout(layout);
    dialog->show();
    connect(btn,&QPushButton::clicked,this,&setDataWidget::PlaneBtnClick);
    dialog->exec();
}

void setDataWidget::PlaneBtnClick(){
    dialog->close();
    if(dis->text().toDouble()<=0||rad->text().toDouble()<=0){
        QMessageBox *messagebox=new QMessageBox();
        messagebox->setText("输入大于0的数");
        messagebox->setIcon(QMessageBox::Warning);
        messagebox->show();
        messagebox->exec();
        return;
    }
    plane=new FittingPlane();
    plane->setRadius(rad->text().toDouble());  // 设置半径
    plane->setDistance(dis->text().toDouble());  // 设置距离阈值
    planeCloud= plane->RANSAC(point,cloudptr);
    if(planeCloud==nullptr){
        qDebug()<<"出现了点云空指针";
        return;
    }
    qDebug()<<planeCloud->size();
    pcl::copyPointCloud(*planeCloud, *fittingPlane);
}

FittingPlane *setDataWidget::getPlane(){
    return plane;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr setDataWidget::getFittingPlane(){
    return fittingPlane;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr setDataWidget::getPlaneCloud(){
    return planeCloud;
}


//圆柱
void setDataWidget::setCylinderData(pcl::PointXYZRGB searchPoint,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
    point=searchPoint;
    cloudptr=cloud;
    dialog = new QDialog(this);
    dialog->resize(400,150);
    dialog->setWindowTitle("设置邻域和阈值");
    layout = new QGridLayout(dialog);
    lab1 = new QLabel("请输入邻域：");
    lab2 = new QLabel("请输入距离阈值：");
    // rad = new QLineEdit();
    // rad->setText("1");
    // dis = new QLineEdit();
    // dis->setText("0.01");
    btn = new QPushButton("确定");
    layout->addWidget(lab1,0,0,1,1);
    layout->addWidget(rad,0,1,1,2);
    layout->addWidget(lab2,1,0,1,1);
    layout->addWidget(dis,1,1,1,2);
    layout->addWidget(btn,2,2,1,1);
    dialog->setLayout(layout);
    dialog->show();
    connect(btn,&QPushButton::clicked,this,&setDataWidget::CylinderBtnClick);
    dialog->exec();
}

void setDataWidget::CylinderBtnClick(){
    dialog->close();
    if(dis->text().toDouble()<=0||rad->text().toDouble()<=0){
        QMessageBox *messagebox=new QMessageBox();
        messagebox->setText("输入大于0的数");
        messagebox->setIcon(QMessageBox::Warning);
        messagebox->show();
        messagebox->exec();
        return;
    }
    cylinder=new FittingCylinder();
    cylinder->setRadius(rad->text().toDouble());  // 设置半径
    cylinder->setDistance(dis->text().toDouble());  // 设置距离阈值
    cylinderCloud= cylinder->RANSAC(point,cloudptr);
    if(cylinderCloud==nullptr){
        qDebug()<<"出现了点云空指针";
        return;
    }
    qDebug()<<cylinderCloud->size();
    pcl::copyPointCloud(*cylinderCloud, *fittingCylinder);
}

FittingCylinder *setDataWidget::getCylinder(){
    return cylinder;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr setDataWidget::getFittingCylinder(){
    return fittingCylinder;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr setDataWidget::getCylinderCloud(){
    return cylinderCloud;
}


//球
void setDataWidget::setSphereData(pcl::PointXYZRGB searchPoint,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
    point=searchPoint;
    cloudptr=cloud;
    dialog = new QDialog(this);
    dialog->resize(400,150);
    dialog->setWindowTitle("设置邻域和阈值");
    layout = new QGridLayout(dialog);
    lab1 = new QLabel("请输入邻域：");
    lab2 = new QLabel("请输入距离阈值：");
    // rad = new QLineEdit();
    // rad->setText("1");
    // dis = new QLineEdit();
    // dis->setText("0.01");
    btn = new QPushButton("确定");
    layout->addWidget(lab1,0,0,1,1);
    layout->addWidget(rad,0,1,1,2);
    layout->addWidget(lab2,1,0,1,1);
    layout->addWidget(dis,1,1,1,2);
    layout->addWidget(btn,2,2,1,1);
    dialog->setLayout(layout);
    dialog->show();
    connect(btn,&QPushButton::clicked,this,&setDataWidget::SphereBtnClick);
    dialog->exec();
}

void setDataWidget::SphereBtnClick(){
    dialog->close();
    if(dis->text().toDouble()<=0||rad->text().toDouble()<=0){
        QMessageBox *messagebox=new QMessageBox();
        messagebox->setText("输入大于0的数");
        messagebox->setIcon(QMessageBox::Warning);
        messagebox->show();
        messagebox->exec();
        return;
    }
    sphere=new FittingSphere();
    sphere->setRadius(rad->text().toDouble());  // 设置半径
    sphere->setDistance(dis->text().toDouble());  // 设置距离阈值
    sphereCloud= sphere->RANSAC(point,cloudptr);
    if(sphereCloud==nullptr){
        qDebug()<<"出现了点云空指针";
        return;
    }
    qDebug()<<sphereCloud->size();
    pcl::copyPointCloud(*sphereCloud, *fittingSphere);
}

FittingSphere *setDataWidget::getSphere(){
    return sphere;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr setDataWidget::getFittingSphere(){
    return fittingSphere;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr setDataWidget::getSphereCloud(){
    return sphereCloud;
}


//圆锥
void setDataWidget::setConeData(pcl::PointXYZRGB searchPoint,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
    point=searchPoint;
    cloudptr=cloud;
    dialog = new QDialog(this);
    dialog->resize(400,150);
    dialog->setWindowTitle("设置邻域和阈值");
    layout = new QGridLayout(dialog);
    lab1 = new QLabel("请输入邻域：");
    lab2 = new QLabel("请输入距离阈值：");
    // rad = new QLineEdit();
    // rad->setText("1");
    // dis = new QLineEdit();
    // dis->setText("0.01");
    btn = new QPushButton("确定");
    layout->addWidget(lab1,0,0,1,1);
    layout->addWidget(rad,0,1,1,2);
    layout->addWidget(lab2,1,0,1,1);
    layout->addWidget(dis,1,1,1,2);
    layout->addWidget(btn,2,2,1,1);
    dialog->setLayout(layout);
    dialog->show();
    connect(btn,&QPushButton::clicked,this,&setDataWidget::ConeBtnClick);
    dialog->exec();
}

void setDataWidget::ConeBtnClick(){
    dialog->close();
    if(dis->text().toDouble()<=0||rad->text().toDouble()<=0){
        QMessageBox *messagebox=new QMessageBox();
        messagebox->setText("输入大于0的数");
        messagebox->setIcon(QMessageBox::Warning);
        messagebox->show();
        messagebox->exec();
        return;
    }
    cone=new FittingCone();
    cone->setRadius(rad->text().toDouble());  // 设置半径
    cone->setDistance(dis->text().toDouble());  // 设置距离阈值
    coneCloud= cone->RANSAC(point,cloudptr);
    if(coneCloud==nullptr){
        qDebug()<<"出现了点云空指针";
        return;
    }
    qDebug()<<coneCloud->size();
    pcl::copyPointCloud(*coneCloud, *fittingCone);
}

FittingCone *setDataWidget::getCone(){
    return cone;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr setDataWidget::getFittingCone(){
    return fittingCone;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr setDataWidget::getConeCloud(){
    return coneCloud;
}


//平面
void setDataWidget::setLineData(pcl::PointXYZRGB searchPoint,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
    point=searchPoint;
    cloudptr=cloud;
    dialog = new QDialog(this);
    dialog->resize(400,150);
    dialog->setWindowTitle("设置邻域和阈值");
    layout = new QGridLayout(dialog);
    lab1 = new QLabel("请输入邻域：");
    lab2 = new QLabel("请输入距离阈值：");
    // rad = new QLineEdit();
    // rad->setText("1");
    // dis = new QLineEdit();
    // dis->setText("0.01");
    btn = new QPushButton("确定");
    layout->addWidget(lab1,0,0,1,1);
    layout->addWidget(rad,0,1,1,2);
    layout->addWidget(lab2,1,0,1,1);
    layout->addWidget(dis,1,1,1,2);
    layout->addWidget(btn,2,2,1,1);
    dialog->setLayout(layout);
    dialog->show();
    connect(btn,&QPushButton::clicked,this,&setDataWidget::LineBtnClick);
    dialog->exec();
}

void setDataWidget::LineBtnClick(){
    dialog->close();
    if(dis->text().toDouble()<=0||rad->text().toDouble()<=0){
        QMessageBox *messagebox=new QMessageBox();
        messagebox->setText("输入大于0的数");
        messagebox->setIcon(QMessageBox::Warning);
        messagebox->show();
        messagebox->exec();
        return;
    }
    line=new FittingLine();
    line->setRadius(rad->text().toDouble());  // 设置半径
    line->setDistance(dis->text().toDouble());  // 设置距离阈值
    lineCloud= line->RANSAC(point,cloudptr);
    if(lineCloud==nullptr){
        qDebug()<<"出现了点云空指针";
        return;
    }
    qDebug()<<lineCloud->size();
    pcl::copyPointCloud(*lineCloud, *fittingLine);
}

FittingLine *setDataWidget::getLine(){
    return line;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr setDataWidget::getFittingLine(){
    return fittingLine;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr setDataWidget::getLineCloud(){
    return lineCloud;
}
