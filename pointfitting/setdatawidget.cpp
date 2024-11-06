#include "setdatawidget.h"
#include "mainwindow.h"
#include "pointfitting/fittingplane.h"

setDataWidget::setDataWidget(QWidget *parent)
    : QWidget{parent}
{
    m_pMainWin=(MainWindow*)parent;
}

void setDataWidget::setPlaneData(){
    p_dialog = new QDialog(nullptr);
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

    connect(p_btn,&QPushButton::clicked,this,&setDataWidget::PlaneBtnClick);
}

void setDataWidget::PlaneBtnClick(){
    m_pMainWin->getPWinFittingPlane()->getRadious()=p_rad->text().toDouble();
    m_pMainWin->getPWinFittingPlane()->getDistance()=p_dis->text().toInt();
    //m_pMainWin->getPWinFittingPlane()->
}
