#include"toolwidget.h"
#include"toolaction.h"
#include"vtkwindow/vtkpresetwidget.h"
#include <QtWidgets/QMainWindow>
#include <QMenu>
#include<QString>

#include<QPainter>
#include<QTableWidget>
#include <QMessageBox>
#include<QMenuBar>
#include <QDesktopServices>

// 打开文件

#include <QFile>
#include <QFileDialog>

// 加载office驱动，导入导出excel

#include <ActiveQt/QAxObject>

// 文本流导出word、pdf

#include <QTextDocument>
#include <QTextCursor>
#include <QTextStream>
#include <QTextTable>

// 转为pdf类型
#include <QPrinter>
#include <QPdfWriter>


#include <QDir>
#include <QStringList>
#include <QFileInfo>
#include<QGridLayout>
#include <QFile>
#include <QTextStream>
#include <QStringList>
#include <QDebug>
#include<QResource>
#include<QLabel>
#include"elementlistwidget.h"
#include"geometry/centitytypes.h"
#include"vtkwindow/vtkwidget.h"
#include <vtkProperty.h>
#include <Eigen/Dense>

#include<QTreeWidgetItem>
//constructor
#include"constructor/planeconstructor.h"
#include"constructor/lineconstructor.h"


#include"constructor/planeconstructor.h"
#include "constructor/circleconstructor.h"
#include"constructor/pointconstructor.h"
#include"constructor/rectangleconstructor.h"
#include"constructor/sphereconstructor.h"
#include"constructor/cylinderconstructor.h"
#include"constructor/coneconstructor.h"
#include"constructor/distanceconstructor.h"
#include"constructor/distanceconstructor.h"
#include"constructor/pointcloudconstructor.h"
#include"constructor/angleconstructor.h"

#include <pcl/point_cloud.h>     // PCL 的点云类
#include <pcl/visualization/pcl_visualizer.h> // PCL 的可视化工具

#include "pointfitting/fittingplane.h"//拟合平面算法
#include "pointfitting/fittingcylinder.h"//拟合圆柱算法
#include "pointfitting/fittingsphere.h"//拟合圆柱算法
#include "pointfitting/fittingcone.h"//拟合圆柱算法
#include "pointfitting/fittingpoint.h"
#include "pointfitting/fittingline.h"
#include "pointfitting/fittingcircle.h"
#include "pointfitting/setdatawidget.h"


#include <QFileDialog>
#include <QString>
#include <QStringList>
#include <QTableWidget>
#include <QHeaderView>
#include <QTableWidgetItem>
#include <QPixmap>
#include <QPainter>
#include <QApplication>

//绘制vtk的各种封闭曲面
#include <vtkCylinderSource.h>//圆柱体
#include<vtkCubeSource.h>//长方体
#include<vtkConeSource.h>//圆锥
#include<vtkSphereSource.h>//球
#include<vtkSelectEnclosedPoints.h>//圈中点的算法


#include <QVTKOpenGLNativeWidget.h>

#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include<vtkPNGWriter.h>
#include <vtkJPEGWriter.h>
#include <vtkTIFFWriter.h>
#include <vtkBMPWriter.h>
#include <vtkWindowToImageFilter.h>

#include<QTextDocumentWriter>
#include<QPrintDialog>


//Pdf保存
#include <QApplication>
#include <QPdfWriter>
#include <QPainter>
//Excelbaocun
//#include <QtXlsx>
//图片保存
#include <QDateTime>



int getImagePaths(const QString& directory, QStringList &iconPaths, QStringList &iconNames);

ToolWidget::ToolWidget(QWidget *parent)
    : QWidget(parent) {


    QVBoxLayout *layout = new QVBoxLayout(this);

    m_pMainWin =(MainWindow*)parent;


    InitOutputFolder();


    resize(400,250);

    m_nSaveActionNum=5;
    m_nConstructActionNum=10;
    m_nFindActionNum=8;
    m_nCoordActionNum=3;
    m_nViewAngleActionNum=4;

    //静态保存图片路径和名称



    save_action_iconpath_list_<<":/component/save/excel.png"<< ":/component/save/pdf.jpg"<< ":/component/save/txt.jpg"<< ":/component/save/word.jpg"<<":/component/save/image.jpg";
    construct_action_iconpath_list_<<":/component/construct/point_.png"<<":/component/construct/line_.png"<<":/component/construct/circle_.png"<<   ":/component/construct/plane_.png"<<  ":/component/construct/rectangle_.png"<<":/component/construct/cylinder_.png"<< ":/component/construct/cone_.png"<< ":/component/construct/sphere_.png"<<":/component/construct/distance.png"<<":/component/construct/pointCloud.png"<<":/component/construct/angle.png";
    find_action_iconpath_list_<<":/component/construct/point.jpg"<<":/component/construct/line.jpg"<<":/component/construct/circle.jpg"<<   ":/component/construct/plan.jpg"<<  ":/component/construct/rectangle.jpg"<<":/component/construct/cylinder.jpg"<< ":/component/construct/cone.jpg"<< ":/component/construct/sphere.jpg";
    coord_action_iconpath_list_<<":/component/coord/create.png"<<  ":/component/coord/spin.jpg"<<":/component/coord/save.png";
    view_angle_action_iconpath_list_<<":/component/viewangle/front.png"<<":/component/viewangle/up.png"<<":/component/viewangle/right.png"<<":/component/viewangle/isometric.png";





    save_action_name_list_<<"excel"<< "pdf"<< "txt"<< "word"<<"image";
    construct_action_name_list_<<"构造点"<<"构造线"<<"构造圆"<<"构造平面"<<"构造矩形"<<"构造圆柱"<<"构造圆锥"<<"构造球形"<<"构造距离"<<"构造点云"<<"构造角度";
    find_action_name_list_<<"识别点"<<"识别线"<<"识别圆"<<"识别平面"<<"识别矩形"<<"识别圆柱"<<"识别圆锥"<<"识别球形";;
    coord_action_name_list_<<"创建坐标系"<<"旋转坐标系"<<"保存坐标系";
    view_angle_action_name_list_<<"主视角"<<"俯视角"<<"侧视角"<<"立体视角";


    m_nSaveActionNum=save_action_name_list_.count();
    m_nConstructActionNum=construct_action_name_list_.count();
    m_nFindActionNum=find_action_name_list_.count();
    m_nCoordActionNum=coord_action_name_list_.count();
    m_nViewAngleActionNum=view_angle_action_name_list_.count();


    save_actions_      =new ToolAction * [m_nSaveActionNum];
    construct_actions_ =new ToolAction * [m_nConstructActionNum];
    find_actions_ =    new ToolAction * [m_nFindActionNum];
    coord_actions_     =new ToolAction * [m_nCoordActionNum];
    view_angle_actions_= new ToolAction * [m_nViewAngleActionNum];


    //创建工具栏
    int allActionNum=  m_nSaveActionNum+m_nConstructActionNum+m_nFindActionNum+m_nCoordActionNum+m_nViewAngleActionNum;

    m_nToolbarNum =(m_nSaveActionNum-1)/SingalToolBarActionNum+(m_nConstructActionNum-1)/SingalToolBarActionNum+
            (m_nCoordActionNum-1)/SingalToolBarActionNum+(m_nFindActionNum-1)/SingalToolBarActionNum
        +(m_nViewAngleActionNum-1)/SingalToolBarActionNum+5;

    toolBars=new QToolBar*[m_nToolbarNum];


    for(int i=0;i<m_nToolbarNum;i++){

        toolBars[i]=new QToolBar(this);
        toolBars[i]->setIconSize(QSize(iconsize,iconsize));
        toolBars[i]->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        toolBars[i]->setStyleSheet(
            "QToolButton {"
            "    border: 2px solid lightgray;" // 浅灰色边框
            "    padding: 5px;" // 内部填充
            "}"
            "QToolButton:pressed {"
            "    border: 2px solid gray;" // 按下状态下的灰色边框
            "}"
            "QToolButton:hover {"
            "    border: 2px solid darkgray;" // 悬浮状态下的深灰色边框
            "}"
            "QToolBar::item {"
            "    margin: 10px;" // 工具栏项之间的间隔
            "}"
            "QToolBar::separator {"
            "    width: 3px;" // 设置分隔符的宽度
            "    background: transparent;" // 分隔符背景颜色
            "}"
            );

    }

    //为工具栏添加QAction
    for(int i=0;i<m_nSaveActionNum;i++){

        ToolAction* action=new ToolAction(this);
        action->setToolActionKind(SaveAction);
        action->setName(save_action_name_list_[i]);
        action->setIcon(QIcon(save_action_iconpath_list_[i]));
        action->setToolTip(save_action_name_list_[i]);
        save_actions_[i]=action;

    }

    for(int i=0;i<m_nConstructActionNum;i++){

        ToolAction* action=new ToolAction(this);
        action->setToolActionKind(ConstructAction);
        action->setName(construct_action_name_list_[i]);
        action->setIcon(QIcon(construct_action_iconpath_list_[i]));
        action->setToolTip(construct_action_name_list_[i]);
        construct_actions_[i]=action;

    }

    for(int i=0;i<m_nCoordActionNum;i++){

        ToolAction* action=new ToolAction(this);
        action->setToolActionKind(CoordAction);
        action->setName(coord_action_name_list_[i]);
        action->setIcon(QIcon(coord_action_iconpath_list_[i]));
        action->setToolTip(coord_action_name_list_[i]);
        coord_actions_[i]=action;

    }

    for(int i=0;i<m_nFindActionNum;i++){

        ToolAction* action=new ToolAction(this);
        action->setToolActionKind(FindAction);
        action->setName(find_action_name_list_[i]);
        action->setIcon(QIcon(find_action_iconpath_list_[i]));
        action->setToolTip(find_action_name_list_[i]);
        find_actions_[i]=action;

    }

    for(int i=0;i<m_nViewAngleActionNum;i++){

        ToolAction* action=new ToolAction(this);
        action->setToolActionKind(ViewAngleAction);
        action->setName(view_angle_action_name_list_[i]);
        action->setIcon(QIcon(view_angle_action_iconpath_list_[i]));
        action->setToolTip(view_angle_action_name_list_[i]);
        view_angle_actions_[i]=action;
    }



    //设置布局
    setLayout(layout);

    //设置工具栏窗口
    createToolWidget();


    //设置QAction信号槽
    connectActionWithF();

}
void ToolWidget::InitOutputFolder(){
    QString currentPath = QCoreApplication::applicationDirPath();
    //获得前两级路径，并在此基础上生成文件夹
    QString ParentPath=getParentPath(2);
    CompareImagePath= ParentPath+"/点云对比图像/screenshot"+"/";

    createFolder(ParentPath+"/点云对比图像/screenshot");
    createFolder(ParentPath+"/输出/word");
    createFolder(ParentPath+"/输出/pdf");
    createFolder(ParentPath+"/输出/image");
    createFolder(ParentPath+"/输出/excel");
    createFolder(ParentPath+"/输出/txt");
}
void ToolWidget::clearToolWidget(){

    //清空
    for(int i=0;i<m_nToolbarNum;i++){
        clearToolBar(toolBars[i]);
    }

}
void ToolWidget::createToolWidget(){
    int toolbar_index=-1;
    int lastToolBar_index=0;


    //layout()->addWidget(new QLabel("识别:"));
    toolbar_index= addFindActions(find_action_name_list_,m_nFindActionNum, toolbar_index);
    for(int i=lastToolBar_index;i<=toolbar_index;i++){
        layout()->addWidget(toolBars[i]);
    }
    lastToolBar_index=toolbar_index+1;


   // layout()->addWidget(new QLabel("构造:"));
    toolbar_index= addConstructActions(construct_action_name_list_,m_nConstructActionNum, toolbar_index);
    for(int i=lastToolBar_index;i<=toolbar_index;i++){
        layout()->addWidget(toolBars[i]);
    }
    lastToolBar_index=toolbar_index+1;


    //layout()->addWidget(new QLabel("保存:"));
    toolbar_index= addSaveActions(save_action_name_list_,m_nSaveActionNum, toolbar_index);
    for(int i=lastToolBar_index;i<=toolbar_index;i++){
        layout()->addWidget(toolBars[i]);
    }
    lastToolBar_index=toolbar_index+1;


    //layout()->addWidget(new QLabel("坐标系:"));
    toolbar_index= addCoordActions(coord_action_name_list_,m_nCoordActionNum, toolbar_index);
    for(int i=lastToolBar_index;i<m_nToolbarNum;i++){
        layout()->addWidget(toolBars[i]);
    }
    lastToolBar_index=toolbar_index+1;

    //layout()->addWidget(new QLabel("视角:"));
    toolbar_index= addViewAngleActions(view_angle_action_name_list_,m_nViewAngleActionNum, toolbar_index);
    for(int i=lastToolBar_index;i<m_nToolbarNum;i++){
        layout()->addWidget(toolBars[i]);
    }


}
void ToolWidget::clearToolBar(QToolBar *toolbar) {
    //找出每一个控件然后移除
    QList<QAction*> actions = toolbar->actions();
    for (QAction *action : actions) {
        toolbar->removeAction(action);
    }
}
ToolWidget::~ToolWidget(){

}
int ToolWidget::addSaveActions(QStringList& action_name_list ,int action_num,int toolbar_index){
    int action_count=0;
    int index_=0;
    for(int i=0;i<action_num;i++){
        //更新工具栏行数
        if(action_count%SingalToolBarActionNum==0){
            toolbar_index++;
        }
        for(index_=0;index_<m_nSaveActionNum;index_++){
            if(action_name_list[i]==save_action_name_list_[index_]){
                action_count++;
                toolBars[toolbar_index]->addAction(save_actions_[index_]);
                toolBars[toolbar_index]->addSeparator();
            }
        }
    }
    return toolbar_index;
}
int ToolWidget::addConstructActions(QStringList& action_name_list ,int action_num,int toolbar_index){
    int action_count=0;
    int index_=0;
    for(int i=0;i<action_num;i++){
        //更新工具栏行数
        if(action_count%SingalToolBarActionNum==0){
            toolbar_index++;
        }
        for(index_=0;index_<m_nConstructActionNum;index_++){
            if(action_name_list[i]==construct_action_name_list_[index_]){
                action_count++;
                toolBars[toolbar_index]->addAction(construct_actions_[index_]);
                toolBars[toolbar_index]->addSeparator();
            }
        }
    }
    return toolbar_index;
}
int ToolWidget::addFindActions(QStringList& action_name_list ,int action_num,int toolbar_index){
    int action_count=0;
    int index_=0;
    for(int i=0;i<action_num;i++){
        //更新工具栏行数
        if(action_count%SingalToolBarActionNum==0){
            toolbar_index++;
        }
        for(index_=0;index_<m_nFindActionNum;index_++){
            if(action_name_list[i]==find_action_name_list_[index_]){
                action_count++;
                toolBars[toolbar_index]->addAction(find_actions_[index_]);
                toolBars[toolbar_index]->addSeparator();
                break;
            }
        }

    }
    return toolbar_index;
}
int ToolWidget::addCoordActions(QStringList& action_name_list ,int action_num,int toolbar_index){
    int action_count=0;
    int index_=0;
    for(int i=0;i<action_num;i++){
        //更新工具栏行数
        if(action_count%SingalToolBarActionNum==0){
            toolbar_index++;
        }
        for(index_=0;index_<m_nCoordActionNum;index_++){
            if(action_name_list[i]==coord_action_name_list_[index_]){
                action_count++;
                toolBars[toolbar_index]->addAction(coord_actions_[index_]);
                toolBars[toolbar_index]->addSeparator();
            }
        }
    }
    return toolbar_index;
}

int ToolWidget::addViewAngleActions(QStringList& action_name_list ,int action_num,int toolbar_index){
    int action_count=0;
    int index_=0;
    for(int i=0;i<action_num;i++){
        //更新工具栏行数
        if(action_count%SingalToolBarActionNum==0){
            toolbar_index++;
        }
        for(index_=0;index_<m_nViewAngleActionNum;index_++){
            if(action_name_list[i]==view_angle_action_name_list_[index_]){
                action_count++;
                toolBars[toolbar_index]->addAction(view_angle_actions_[index_]);
                toolBars[toolbar_index]->addSeparator();
            }
        }
    }
    return toolbar_index;
}


QStringList* ToolWidget::getSaveActionNames(){

    return &save_action_name_list_;

}

QStringList* ToolWidget:: getConstructActionNames(){

    return &construct_action_name_list_;

}

QStringList* ToolWidget::getFindActionNames(){

    return &find_action_name_list_;

}

QStringList* ToolWidget::getCoordActionNames(){

    return &coord_action_name_list_;

}
QStringList* ToolWidget::getViewAngleActionNames(){

    return &view_angle_action_name_list_;

}




int ToolWidget::getToolbarNum(){

    return m_nToolbarNum;

}

int ToolWidget::getSaveActionNum(){

    return m_nSaveActionNum;

}

int ToolWidget::getConstructActionNum(){

    return m_nConstructActionNum;

}

int ToolWidget::getCoordActionNum(){

    return m_nCoordActionNum;

}

int ToolWidget::getFindActionNum(){

    return m_nFindActionNum;

}
int ToolWidget::getViewAngleActionNum(){

    return m_nViewAngleActionNum;

}

void ToolWidget::connectActionWithF(){



    //识别
    connect(find_actions_[find_action_name_list_.indexOf("识别点")],&QAction::triggered,this,&   ToolWidget::onFindPoint);
    connect(find_actions_[find_action_name_list_.indexOf("识别线")],&QAction::triggered,this,&   ToolWidget::onFindLine);
    connect(find_actions_[find_action_name_list_.indexOf("识别圆")],&QAction::triggered,this,&   ToolWidget::onFindCircle);
    connect(find_actions_[find_action_name_list_.indexOf("识别平面")],&QAction::triggered,this,&  ToolWidget:: onFindPlane);
    connect(find_actions_[find_action_name_list_.indexOf("识别矩形")],&QAction::triggered,this,&   ToolWidget::onFindRectangle);
    connect(find_actions_[find_action_name_list_.indexOf("识别圆柱")],&QAction::triggered,this,&   ToolWidget::onFindCylinder);
    connect(find_actions_[find_action_name_list_.indexOf("识别圆锥")],&QAction::triggered,this,&   ToolWidget::onFindCone);
    connect(find_actions_[find_action_name_list_.indexOf("识别球形")],&QAction::triggered,this,&   ToolWidget::onFindSphere);

    //构造
    connect(construct_actions_[construct_action_name_list_.indexOf("构造点")],&QAction::triggered,this,& ToolWidget::onConstructPoint);
    connect(construct_actions_[construct_action_name_list_.indexOf("构造线")],&QAction::triggered,this,&ToolWidget::onConstructLine);
    connect(construct_actions_[construct_action_name_list_.indexOf("构造圆")],&QAction::triggered,this,&ToolWidget::onConstructCircle);
    connect(construct_actions_[construct_action_name_list_.indexOf("构造平面")],&QAction::triggered,this,& ToolWidget::onConstructPlane);
    connect(construct_actions_[construct_action_name_list_.indexOf("构造矩形")],&QAction::triggered,this,& ToolWidget::onConstructRectangle);
    connect(construct_actions_[construct_action_name_list_.indexOf("构造圆柱")],&QAction::triggered,this,&  ToolWidget::onConstructCylinder);
    connect(construct_actions_[construct_action_name_list_.indexOf("构造圆锥")],&QAction::triggered,this,&  ToolWidget::onConstructCone);
    connect(construct_actions_[construct_action_name_list_.indexOf("构造球形")],&QAction::triggered,this,&  ToolWidget::onConstructSphere);
    connect(construct_actions_[construct_action_name_list_.indexOf("构造距离")],&QAction::triggered,this,&  ToolWidget::onConstructDistance);
    connect(construct_actions_[construct_action_name_list_.indexOf("构造点云")],&QAction::triggered,this,&  ToolWidget::onConstructPointCloud);
    connect(construct_actions_[construct_action_name_list_.indexOf("构造角度")],&QAction::triggered,this,&  ToolWidget::onConstructAngle);

    // //保存
     connect(save_actions_[save_action_name_list_.indexOf("excel")],&QAction::triggered,this,&  ToolWidget::onSaveExcel);
    // connect(save_actions_[save_action_name_list_.indexOf("word")],&QAction::triggered,this,&  ToolWidget::onSaveWord);
    // connect(save_actions_[save_action_name_list_.indexOf("txt")],&QAction::triggered,this,&  ToolWidget::onSaveTxt);
    // connect(save_actions_[save_action_name_list_.indexOf("pdf")],&QAction::triggered,this,&  ToolWidget::onSavePdf);
    // connect(save_actions_[save_action_name_list_.indexOf("image")],&QAction::triggered,this,&  ToolWidget::onSaveImage);

    //打开
    //connect(save_actions_[save_action_name_list_.indexOf("excel")], &QAction::triggered, this, &ToolWidget::onOpenExcel);
    connect(save_actions_[save_action_name_list_.indexOf("word")], &QAction::triggered, this, &ToolWidget::onOpenWord);
    connect(save_actions_[save_action_name_list_.indexOf("txt")], &QAction::triggered, this, &ToolWidget::onOpenTxt);
    connect(save_actions_[save_action_name_list_.indexOf("pdf")], &QAction::triggered, this, &ToolWidget::onOpenPdf);
    connect(save_actions_[save_action_name_list_.indexOf("image")], &QAction::triggered, this, &ToolWidget::onOpenImage);

    // connect(save_actions_[save_action_name_list_.indexOf("txt")],&QAction::triggered,this,&  ToolWidget::onSaveTxt);


    //坐标系
    connect(coord_actions_[coord_action_name_list_.indexOf("创建坐标系")],&QAction::triggered,this,[&](){
        tool_widget::onCreateCoord();
        m_pMainWin->on2dCoordOriginAuto(); //创建临时坐标系
    });
    connect(coord_actions_[coord_action_name_list_.indexOf("旋转坐标系")],&QAction::triggered,this,[&](){
        tool_widget::onSpinCoord();
        m_pMainWin->on2dCoordSetRightX(); // x轴摆正
    });
    connect(coord_actions_[coord_action_name_list_.indexOf("保存坐标系")],&QAction::triggered,this,[&](bool){
        tool_widget::onSaveCoord();
        m_pMainWin->on2dCoordSave();
    });
    //视角
    connect(view_angle_actions_[view_angle_action_name_list_.indexOf("主视角")],&QAction::triggered,this,[&](){
        tool_widget::onFrontViewAngle();
        m_pMainWin->onFrontViewClicked();
    });
    connect(view_angle_actions_[view_angle_action_name_list_.indexOf("俯视角")],&QAction::triggered,this, [&](){
        tool_widget::onUpViewAngle();
        m_pMainWin->onTopViewClicked();
    });
    connect(view_angle_actions_[view_angle_action_name_list_.indexOf("侧视角")],&QAction::triggered,[&](){
        tool_widget::onRightViewAngle();
        m_pMainWin->onRightViewClicked();
    });
    connect(view_angle_actions_[view_angle_action_name_list_.indexOf("立体视角")],&QAction::triggered,[&](){
        tool_widget::onIsometricViewAngle();
        m_pMainWin->onIsometricViewClicked();
    });

}



//Find
namespace tool_widget{
void onFrontViewAngle(){qDebug()<<"点击了主视角";}
void onIsometricViewAngle(){qDebug()<<"点击了立体视角";}
void onRightViewAngle(){qDebug()<<"点击了侧视角";}
void onUpViewAngle(){qDebug()<<"点击了俯视角";}
//Coord
void   onCreateCoord(){qDebug()<<"点击了创建坐标系";}
void   onSpinCoord(){qDebug()<<"点击了旋转坐标系";}
void   onSaveCoord(){qDebug()<<"点击了保存坐标系";}
}

//open
void ToolWidget::onOpenExcel() {
    if (!QDesktopServices::openUrl(QUrl::fromLocalFile(lastCreatedExcelFile))) {
        QString log="无法打开Excel文件";
        m_pMainWin->getPWinVtkPresetWidget()->setWidget(log);
    }
}

void ToolWidget::onOpenWord() {
    if (!QDesktopServices::openUrl(QUrl::fromLocalFile(lastCreatedWordFile))) {
        QString log="无法打开Word文件";
        m_pMainWin->getPWinVtkPresetWidget()->setWidget(log);
    }
}

void ToolWidget::onOpenTxt() {
    if (!QDesktopServices::openUrl(QUrl::fromLocalFile(lastCreatedTxtFile))) {
        QString log="无法打开Txt文件";
        m_pMainWin->getPWinVtkPresetWidget()->setWidget(log);
    }
}

void ToolWidget::onOpenPdf() {
    if (!QDesktopServices::openUrl(QUrl::fromLocalFile(lastCreatedPdfFile))) {
        QString log="无法打开Pdf文件";
        m_pMainWin->getPWinVtkPresetWidget()->setWidget(log);
    }
}

void ToolWidget::onOpenImage() {
    if (!QDesktopServices::openUrl(QUrl::fromLocalFile(lastCreatedImageFileFront))) {
        QString log="无法打开Image文件";
        m_pMainWin->getPWinVtkPresetWidget()->setWidget(log);
    }
    if (!QDesktopServices::openUrl(QUrl::fromLocalFile(lastCreatedImageFileTop))) {
        ;// QString log="无法打开Image(Top)文件";
        // m_pMainWin->getPWinVtkPresetWidget()->setWidget(log);
    }
    if (!QDesktopServices::openUrl(QUrl::fromLocalFile(lastCreatedImageFileRight))) {
        ;// QString log="无法打开Image(Right)文件";
        // m_pMainWin->getPWinVtkPresetWidget()->setWidget(log);
    }
}

//Save
//从列表中提取数据转换为 QList<QList<QString>> 类型的二维列表
void   ExtractData(QVector<CEntity *>& entitylist,QList<QList<QString>>& dataAll){
    for (int i = 0; i < entitylist.size(); i++) {
        QList<QString> inList;
        CEntity* entity=entitylist[i];
        if(entity->GetUniqueType()==enPoint){
            continue;
            // CPoint* point=(CPoint*)entity;
            // CPosition position=point->GetPt();
            // inList<<"点";
            // inList<<point->m_strCName;
            // inList<<"坐标:("+QString::number(position.x, 'f', 6)+","+QString::number(position.y, 'f', 6)+","+QString::number(position.z, 'f', 6)+")";
        }else if(entity->GetUniqueType()==enCircle){
            continue;
            // CCircle* circle=(CCircle*)entity;
            // CPosition position=circle->getCenter();
            // inList<<"圆"<<circle->m_strAutoName;
            // inList<<"中心:("+QString::number(position.x, 'f', 6)+","+QString::number(position.y, 'f', 6)+","+QString::number(position.z, 'f', 6)+")";
            // inList<<"直径D:"+QString::number(circle->getDiameter(),'f',6);

        }else if(entity->GetUniqueType()==enSphere){
            continue;
            // CSphere* sphere=(CSphere*)entity;
            // CPosition position=sphere->getCenter();
            // inList<<"球"<<sphere->m_strAutoName;
            // inList<<"中心:("+QString::number(position.x, 'f', 6)+","+QString::number(position.y, 'f', 6)+","+QString::number(position.z, 'f', 6)+")";
            // inList<<"直径D:"+QString::number(sphere->getDiameter(),'f',6);
        }else if(entity->GetUniqueType()==enPlane){
            continue;
            // CPlane* plane=(CPlane*)entity;
            // CPosition position=plane->getCenter();
            // QVector4D normal,dir_long_edge;
            // normal=plane->getNormal();
            // dir_long_edge=plane->getDir_long_edge();
            // inList<<"平面";
            // inList<<plane->m_strAutoName;
            // inList<<"中心:("+QString::number(position.x, 'f', 6)+","+QString::number(position.y, 'f', 6)+","+QString::number(position.z, 'f', 6)+")";
            // inList<<"法线:("+QString::number(normal.x(), 'f', 6)+","+QString::number(normal.y(), 'f', 6)+","+QString::number(normal.z(), 'f', 6)+")";
            // inList<<"边向量:("+QString::number(dir_long_edge.x(), 'f', 6)+","+QString::number(dir_long_edge.y(), 'f', 6)+","+QString::number(dir_long_edge.z(), 'f', 6)+")";
            // inList<<"长:"+QString::number(plane->getLength(), 'f', 6)<<" 宽:"+QString::number(plane->getWidth(), 'f', 6);

        }else if(entity->GetUniqueType()==enCone){
            continue;
            // CCone* cone=(CCone*)entity;

            // CPosition position=cone->getVertex();
            // QVector4D axis;
            // axis=cone->getAxis();
            // inList<<"圆锥";
            // inList<<cone->m_strAutoName;
            // inList<<"顶点:("+QString::number(position.x, 'f', 6)+","+QString::number(position.y, 'f', 6)+","+QString::number(position.z, 'f', 6)+")";
            // inList<<"轴线向量:("+QString::number(axis.x(), 'f', 6)+","+QString::number(axis.y(), 'f', 6)+","+QString::number(axis.z(), 'f', 6)+")";
            // inList<<"高:"+QString::number(cone->getHeight(), 'f', 6)<<" 弧度:"+QString::number(cone->getRadian(), 'f', 6)<<" 圆锥高:"+QString::number(cone->getCone_height(), 'f', 6);
        }
        else if(entity->GetUniqueType()==enCylinder){
            continue;
            // CCylinder* cylinder=(CCylinder*)entity;
            // CPosition position=cylinder->getBtm_center();
            // QVector4D axis;
            // axis=cylinder->getAxis();
            // inList<<"圆柱";
            // inList<<cylinder->m_strAutoName;
            // inList<<"底面中心:("+QString::number(position.x, 'f', 6)+","+QString::number(position.y, 'f', 6)+","+QString::number(position.z, 'f', 6)+")";
            // inList<<"轴线向量:("+QString::number(axis.x(), 'f', 6)+","+QString::number(axis.y(), 'f', 6)+","+QString::number(axis.z(), 'f', 6)+")";
            // inList<<"高:"+QString::number(cylinder->getHeight(),'f',6)<<" 直径:"+QString::number(cylinder->getDiameter(),'f',6);

        }else if(entity->GetUniqueType()==enLine){
            continue;
            // CLine* line=(CLine*)entity;
            // CPosition position1,position2;
            // position1=line->getPosition1();
            // position2=line->getPosition2();
            // inList<<"线";
            // inList<<line->m_strCName;
            // inList<<"起点：("+QString::number(position1.x, 'f', 6)+","+QString::number(position1.y, 'f', 6)+","+QString::number(position1.z, 'f', 6)+")";
            // inList<<"终点：("+QString::number(position2.x, 'f', 6)+","+QString::number(position2.y, 'f', 6)+","+QString::number(position2.z, 'f', 6)+")";
        }else if(entity->GetUniqueType()==enDistance){
            CDistance* Distance=(CDistance*) entity;
            inList<<"距离";
            inList<<Distance->m_strCName;
            inList<<QString::number(Distance->getdistance(),'f',6);
            inList<<"上公差"+QString::number(Distance->getUptolerance(),'f',6);
            inList<<"下公差"+QString::number(Distance->getUndertolerance(),'f',6);
        }else if(entity->GetUniqueType()==enPointCloud){
            continue;
            // CPointCloud* PointCloud=(CPointCloud*) entity;
            // inList<<"点云";
            // inList<<PointCloud->m_strCName;
        }
        else if (entity->GetUniqueType()==enAngle){
            CAngle* Angle=(CAngle*)entity;
            inList<<"角度";
            inList<<Angle->m_strCName;
            inList<<QString::number(Angle->getAngleValue(),'f',6);
            inList<<"上公差"+QString::number(Angle->getUptolerance(),'f',6);
            inList<<"下公差"+QString::number(Angle->getUndertolerance(),'f',6);
        }
        dataAll.append(inList);
    }
}


void insertImageIntoPdf(const QString &imagePath, const QString &pdfPath) {
    // 创建一个新的QTextDocument
    QTextDocument doc;
    QTextCursor cursor(&doc);

    // 在QTextDocument中插入图片
    QTextImageFormat imageFormat;
    imageFormat.setName(imagePath);
    cursor.insertImage(imageFormat);

    // 设置打印机以输出到PDF
    QPrinter printer(QPrinter::HighResolution);
    printer.setOutputFormat(QPrinter::PdfFormat);
    printer.setOutputFileName(pdfPath);

    // 打印文档到PDF文件
    doc.print(&printer);
}





void   ToolWidget::onSavePdf(){

   QString path= getOutputPath("pdf");
   QString name=getTimeString();

   QString filePath=path+"/"+name+".pdf";
   // QString filePath = QFileDialog::getSaveFileName(nullptr, "Save PDF", "", "PDF Files (*.pdf)");
   // if (filePath.isEmpty()) {
   //     return; // 用户取消了保存操作
   // }

   QPdfWriter pdfWriter(filePath);
   pdfWriter.setPageSize(QPageSize::A4);
   pdfWriter.setResolution(300);

   QPainter painter(&pdfWriter);

   // 插入文字
   painter.setFont(QFont("Arial", 12));
   painter.drawText(100, 100, "3dins");

   // 插入纯文本数据
   auto& entitylist = m_pMainWin->m_EntityListMgr->getEntityList();
   QList<QList<QString>> dataAll;
   QList<QString> header;
   header << "类型" << "名称" << "数据1" << "数据2" << "数据3" << "数据4" << "数据5";
   dataAll.append(header);
   ExtractData(entitylist, dataAll);

   int yPos = 150; // 初始y坐标
   for (int i = 0; i < dataAll.size(); ++i) {
       QString line;
       for (int j = 0; j < dataAll[i].size(); ++j) {
           line += dataAll[i][j] + " "; // 用空格分隔列数据
       }
       painter.drawText(100, yPos, line);
       yPos += 80; // 每行间隔80像素
   }

   // 插入图片
   QImage image;
   for (int i = 0; i < imagePaths.size(); ++i) {
       image = QImage(imagePaths[i]);
       if (!image.isNull()) {
           // 检查图片高度是否会超出页面
           if (yPos + image.height() > pdfWriter.height()) {
               // 超出页面高度时，添加新的一页
               pdfWriter.newPage();
               yPos = 0; // 重置y坐标
           }
           painter.drawImage(100, yPos, image);
           yPos += image.height() + 20; // 图片下方留出20像素的间隔
       }
   }

   painter.end();
   QString logInfo="Pdf保存成功";
   m_pMainWin->getPWinVtkPresetWidget()->setWidget(logInfo);
   lastCreatedPdfFile=filePath;

}
// void ToolWidget::onSaveExcel() {
//     QString path = getOutputPath("xlsx");
//     QString name = getTimeString();
//     QString filePath = path + "/" + name + ".xlsx";

//     QStringList headers;
//     headers << "类型" << "名称" << "数据1" << "数据2" << "数据3" << "数据4" << "数据5";
//     QList<QList<QString>> dataAll;
//     auto& entitylist = m_pMainWin->m_EntityListMgr->getEntityList();
//     ExtractData(entitylist, dataAll);

//     QXlsx::Document xlsx;
//     xlsx.write("A1", "entitylist 数据输出");
//     xlsx.mergeCells("A1:G1");
//     xlsx.setRowHeight(1, 30);

//     // 写入列标题
//     for (int i = 0; i < headers.size(); ++i) {
//         xlsx.write(2, i + 1, headers[i]);
//     }

//     // 写入数据
//     for (int row = 0; row < dataAll.size(); ++row) {
//         for (int col = 0; col < dataAll[row].size(); ++col) {
//             xlsx.write(row + 3, col + 1, dataAll[row][col]);
//         }
//     }

//     // 保存文件
//     if (xlsx.saveAs(filePath)) {
//         QMessageBox::information(nullptr, "提示", "保存成功");
//     } else {
//         QMessageBox::warning(nullptr, "提示", "保存失败");
//     }
// }

void   ToolWidget::onSaveExcel(){
    QString path= getOutputPath("xlsx");
    QString name=getTimeString();

    QString filePath=path+"/"+name+".xlsx";
    // QString filePath = QFileDialog::getSaveFileName(nullptr, QString("Save As"), "", QString("Excel(*.xlsx *.xls)"));
    // if (filePath.isEmpty()){
    //     return ;
    // }

    QStringList headers;
    headers << "类型" << "名称" << "数据1" << "数据2" << "数据3"<<"数据4"<<"数据5" ;
    int col = headers.size();
    QList<QList<QString>> dataAll;
    auto& entitylist = m_pMainWin->m_EntityListMgr->getEntityList();
    //dataAll.append(headers);
    ExtractData(entitylist,dataAll);                              //将数据传入dataAll中

    QAxObject excel("Excel.Application");						  //加载Excel驱动
    excel.dynamicCall("SetVisible (bool Visible)", "false");	  //不显示窗体
    excel.setProperty("DisplayAlerts", true);					  //不显示任何警告信息。如果为true那么在关闭是会出现类似“文件已修改，是否保存”的提示

    QAxObject *workBooks = excel.querySubObject("WorkBooks");	  //获取工作簿集合
    workBooks->dynamicCall("Add");								  //新建一个工作簿
    QAxObject *workBook = excel.querySubObject("ActiveWorkBook"); //获取当前工作簿
    //QAxObject *workBook = excel.querySubObject("Open(QString&)", filePath); //获取当前工作簿
    QAxObject *workSheet = workBook->querySubObject("Sheets(int)", 1); //设置为 获取第一页 数据

    // 大标题行
    QAxObject *cell;
    cell = workSheet->querySubObject("Cells(int,int)", 1, 1);
    cell->dynamicCall("SetValue(const QString&)", "entitylist 数据输出");
    cell->querySubObject("Font")->setProperty("Size", 11);
    // 合并标题行
    QString cellTitle;
    cellTitle.append("A1:");
    cellTitle.append(QChar(col - 1 + 'A'));
    cellTitle.append(QString::number(1));
    QAxObject *range = workSheet->querySubObject("Range(const QString&)", cellTitle);
    range->setProperty("WrapText", true);
    range->setProperty("MergeCells", true);
    range->setProperty("HorizontalAlignment", -4108);
    range->setProperty("VertivcalAlignment", -4108);

    // 行高
    workSheet->querySubObject("Range(const QString&)", "1:1")->setProperty("RowHeight", 30);

    //列标题
    QString lastCars = QChar('A');
    for (int i = 0; i < col; i++)
    {
        // excel表格 A:A第一列到第一列,B:B第二列到第二列
        // A B C D...X Y Z AA AB AC...AX AY AZ BA BB BC...
        QString columnName;
        QString cars;
        if (i < 26) {
            // 列数少于26个字母A B C D...X Y Z
            cars = QChar(i + 'A');
        } else {
            // 列数大于26个字母 AA AB AC...AX AY AZ BA BB BC...
            cars = QChar(i / 26 - 1 + 'A');
            cars.append(QChar(i % 26 + 'A'));
        }
        columnName = cars + ":" + cars;
        lastCars = cars;
        // 有大标题，列标题从第二行开始("Cells(int, int)", 2, i+1)
        // 无大标题，列标题从第一行开始("Cells(int, int)", 1, i+1)
        QAxObject *col = workSheet->querySubObject("Columns(const QString&)", columnName);
        QAxObject *cell = workSheet->querySubObject("Cells(int, int)", 2, i+1);
        cell->dynamicCall("SetValue(const QString&)", headers[i]);
        cell->querySubObject("Font")->setProperty("Bold", true);
        cell->querySubObject("Interior")->setProperty("Color", QColor(191, 191, 191));
        cell->setProperty("WrapText", true);						//内容过多，自动换行
        cell->setProperty("HorizontalAlignment", -4108);
        cell->setProperty("VertivcalAlignment", -4108);
    }

    //处理数据
    int curRow = 3;
    foreach(QList<QString> inLst, dataAll) {
        for (int j = 0; j < inLst.size(); j++) {
            // ("Cells(int, int)", row, col)单元格的行和列从开始
            QAxObject *cell = workSheet->querySubObject("Cells(int, int)", curRow, j+1);
            cell->dynamicCall("SetValue(const QString&)", inLst[j]);
            QAxObject* border = cell->querySubObject("Borders");
            border->setProperty("Color", QColor(0, 0, 0));		 //设置单元格边框色（黑色）
        }
        curRow++;
    }

    // 自动调整列宽
    for (int i = 0; i < col; i++) {
        QString columnName;
        QString cars;
        if (i < 26) {
            cars = QChar(i + 'A');
        } else {
            cars = QChar(i / 26 - 1 + 'A');
            cars.append(QChar(i % 26 + 'A'));
        }
        columnName = cars + ":" + cars;
        QAxObject *column = workSheet->querySubObject("Columns(const QString&)", columnName);
        column->dynamicCall("AutoFit()");
    }

    //保存至filepath，注意一定要用QDir::toNativeSeparators将路径中的"/"转换为"\"，不然一定保存不了。
    workBook->dynamicCall("SaveAs(const QString&)", QDir::toNativeSeparators(filePath));
    workBook->dynamicCall("Close()");	//关闭工作簿
    excel.dynamicCall("Quit()");		//关闭excel
    QString logInfo="Excel保存成功";
    m_pMainWin->getPWinVtkPresetWidget()->setWidget(logInfo);
    lastCreatedExcelFile=filePath;
}

void ToolWidget::onSaveTxt(){
    QString filePath;
    IsAuto=true;
    if(IsAuto){
        QString desktopPath = QStandardPaths::writableLocation(QStandardPaths::DesktopLocation);
        if (desktopPath.isEmpty()) {
            qWarning() << "无法获取桌面路径";
            return;
        }

        // 生成文件名
        QString fileName = "Entities_List";
        auto& entitylist = m_pMainWin->m_EntityListMgr->getEntityList();
        for (int i = 0; i < entitylist.size(); i++) {
            CEntity* entity = entitylist[i];
            if (i > 0) {
                fileName += "_";
            }
            fileName += entity->m_strAutoName;
        }
        fileName += ".txt";

        // 完整的文件路径
        filePath = desktopPath + "/" + fileName;
    }else{
        filePath = QFileDialog::getSaveFileName(nullptr, QString("Save As"), "", QString("txt(*.txt )"));
        if (filePath.isEmpty()){
            return ;
        }

        if (QFileInfo(filePath).suffix().isEmpty())
            filePath.append(".txt");
    }
    //先覆盖一下
    QString path= getOutputPath("txt");
    QString name=getTimeString();

     filePath=path+"/"+name+".txt";


    // 创建 QFile 对象
    QFile file(filePath);

    // 打开文件以进行写入
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        qWarning() << "无法打开文件:" << file.errorString();
        return;
    }

    // 创建 QTextStream 对象
    QTextStream out(&file);
    auto& objectlist = m_pMainWin->m_ObjectListMgr->getObjectList();
    // 写入内容

    for(int i=0;i<objectlist.size();i++){
        CObject* object=objectlist[i];
        if(object->GetUniqueType()==enPoint){
            continue;
            // CPoint * point=(CPoint*)object;
            // CPosition position =point->GetPt();
            // out<<"类型：点 名称:"<<point->m_strAutoName<<Qt::endl;
            // out<<"坐标:("<<position.x<<","<<position.y<<","<<position.z<<")"<<Qt::endl;
            // out<<Qt::endl;
        }else if(object->GetUniqueType()==enCircle){
            continue;
            // CCircle* circle=(CCircle*)object;
            // CPosition position=circle->getCenter();
            // out<<"类型：圆 名称:"<<circle->m_strAutoName<<Qt::endl;
            // out<<"中心:("<<position.x<<","<<position.y<<","<<position.z<<")"<<Qt::endl;
            // out<<"直径D:"<<circle->getDiameter();
            // out<<Qt::endl;
            // out<<Qt::endl;

        }else if(object->GetUniqueType()==enSphere){
            continue;
            // CSphere* sphere=(CSphere*)object;
            // CPosition position=sphere->getCenter();
            // out<<"类型：球 名称:"<<sphere->m_strAutoName<<Qt::endl;
            // out<<"中心:("<<position.x<<","<<position.y<<","<<position.z<<")"<<Qt::endl;
            // out<<"直径D:"<<sphere->getDiameter();
            // out<<Qt::endl;
            // out<<Qt::endl;

        }else if(object->GetUniqueType()==enPlane){
            continue;
            // CPlane* plane=(CPlane*)object;
            // CPosition position=plane->getCenter();
            // QVector4D normal,dir_long_edge;
            // normal=plane->getNormal();
            // dir_long_edge=plane->getDir_long_edge();
            // out<<"类型：平面 名称:"<<plane->m_strAutoName<<Qt::endl;
            // out<<"中心:("<<position.x<<","<<position.y<<","<<position.z<<")"<<Qt::endl;
            // out<<"法线:("<<normal.x()<<","<<normal.y()<<","<<normal.z()<<")"<<Qt::endl;
            // out<<"边向量:("<<dir_long_edge.x()<<","<<dir_long_edge.y()<<","<<dir_long_edge.z()<<")"<<Qt::endl;
            // out<<"长:"<<plane->getLength()<<" 宽:"<<plane->getWidth()<<Qt::endl;
            // out<<Qt::endl;

        }else if(object->GetUniqueType()==enCone){
            continue;
            // CCone* cone=(CCone*)object;

            // CPosition position=cone->getVertex();
            // QVector4D axis;
            // axis=cone->getAxis();
            // out<<"类型：圆锥 名称:"<<cone->m_strAutoName<<Qt::endl;
            // out<<"顶点:("<<position.x<<","<<position.y<<","<<position.z<<")"<<Qt::endl;
            // out<<"轴线向量:("<<axis.x()<<","<<axis.y()<<","<<axis.z()<<")"<<Qt::endl;
            // out<<"高:"<<cone->getHeight()<<" 弧度:"<<cone->getRadian()<<" 圆锥高:"<<cone->getCone_height()<<Qt::endl;
            // out<<Qt::endl;

        }else if(object->GetUniqueType()==enCylinder){
            continue;
            // CCylinder* cylinder=(CCylinder*)object;
            // CPosition position=cylinder->getBtm_center();
            // QVector4D axis;
            // axis=cylinder->getAxis();
            // out<<"类型：圆柱 名称:"<<cylinder->m_strAutoName<<Qt::endl;
            // out<<"底面中心:("<<position.x<<","<<position.y<<","<<position.z<<")"<<Qt::endl;
            // out<<"轴线向量:("<<axis.x()<<","<<axis.y()<<","<<axis.z()<<")"<<Qt::endl;
            // out<<"高:"<<cylinder->getHeight()<<" 直径:"<<cylinder->getDiameter()<<Qt::endl;
            // out<<Qt::endl;

        }else if(object->GetUniqueType()==enLine){
            continue;
            // CLine* line=(CLine*)object;
            // CPosition position1,position2;
            // position1=line->getPosition1();
            // position2=line->getPosition2();
            // out<<"类型：线 名称:"<<line->m_strAutoName<<Qt::endl;
            // out<<"起点:("<<position1.x<<","<<position1.y<<","<<position1.z<<")"<<Qt::endl;
            // out<<"终点:("<<position2.x<<","<<position2.y<<","<<position2.z<<")"<<Qt::endl;
            // out<<Qt::endl;

        }else if(object->GetUniqueType()==enDistance){
            CDistance* Distance=(CDistance*) object;
            out<<"类型：距离 名称："<<Distance->m_strCName<<Qt::endl;
            out<<"大小："<<Distance->getdistance()<<"  上公差："<<Distance->getUptolerance()<<"  下公差："<<Distance->getUndertolerance()<<Qt::endl;
            QString qua;
            if(Distance->judge())
            {
                qua="合格";
            }else{
                qua="不合格";
            }
            out<<"是否合格："<<qua<<Qt::endl;
            out<<Qt::endl;
        }else if(object->GetUniqueType()==enPointCloud){
            continue;
            // CPointCloud* PointCloud=(CPointCloud*) object;
            // out<<"类型：距离 名称:"<<PointCloud->m_strCName<<Qt::endl;
            // out<<Qt::endl;
        }else if(object->GetUniqueType()==enAngle){
            CAngle* Angle=(CAngle*)object;
            out<<"类型：角度 名称："<<Angle->m_strCName<<Qt::endl;
            out<<"大小："<<Angle->getAngle()<<Qt::endl;
            QString qua;
            if(Angle->judge())
            {
                qua="合格";
            }else{
                qua="不合格";
            }
            out<<"是否合格："<<qua<<Qt::endl;
            out<<Qt::endl;
        }
        ;
    }
    // 关闭文件
    file.close();
    QString logInfo="Txt保存成功";
    m_pMainWin->getPWinVtkPresetWidget()->setWidget(logInfo);
    lastCreatedTxtFile=filePath;
}




void   ToolWidget::onSaveWord(){
    QString path= getOutputPath("word");
    QString name=getTimeString();

    QString filePath=path+"/"+name+".docx";
    // QString filePath = QFileDialog::getSaveFileName(nullptr, QString("Save As"), "", QString("word(*.doc *.docx)"));
    // if (filePath.isEmpty()) {
    //     return;
    // }

    QStringList headers;
    headers << "类型" << "名称" << "数据1" << "数据2" << "数据3" << "数据4" << "数据5";
    auto& entitylist = m_pMainWin->m_EntityListMgr->getEntityList();
    int col = headers.size();
    int row = entitylist.size();
    QList<QList<QString>> dataAll;
    ExtractData(entitylist, dataAll);

    // 创建一个QTextDocument对象
    QTextDocument doc;

    // 创建一个QTextCursor对象
    QTextCursor cursor(&doc);

    // 标题和参数信息
    cursor.insertText("entitylist列表输出\n\n");

    // 插入表头
    for (const QString& header : headers) {
        cursor.insertText(header + "\t");
    }
    cursor.insertBlock();

    // 插入数据
    for (const QList<QString>& rowData : dataAll) {
        for (const QString& cellData : rowData) {
            cursor.insertText(cellData + "\t");
        }
        cursor.insertBlock();
    }

    // vtkRenderWindow渲染窗口的代码不变

    // 将QTextDocument的内容保存为纯文本文件
    QFile file(filePath);
    if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QTextStream stream(&file);
        stream << doc.toPlainText();
        file.close();
        QString logInfo="Word保存成功";
        m_pMainWin->getPWinVtkPresetWidget()->setWidget(logInfo);
    }
    lastCreatedWordFile=filePath;

}



static void WrongWidget(QString message,QString moreMessage="空");
void   ToolWidget::onSaveImage(){
    //得到路径名称
    QString filter = "PNG (*.png);;JPEG (*.jpg *.jpeg);;TIFF (*.tif *.tiff);;BMP (*.bmp)";
    QString fileName;
    if (!IsAuto) {
        // 如果不是自动保存，弹出文件选择对话框
        fileName = QFileDialog::getSaveFileName(this, "Save Screenshot", "", filter, &filter);
    } else {
        // 如果是自动保存，设置默认保存路径
        fileName = "C:/Users/Lenovo/Desktop/imageSave";
    }
    QString path= getOutputPath("image");
    QString name=getTimeString();
    fileName=path+"/"+name+".png";
    QFileInfo fileInfo(fileName);

    QString filePath = fileInfo.absolutePath(); // 文件的路径

    QString baseName = fileInfo.baseName();//文件的基本名

    QString suffix = fileInfo.suffix();//文件的后缀

    // 生成三个新的文件名
    QString fileNameFront =filePath+"/"+ baseName + "_Front." + suffix;
    QString fileNameTop =filePath+"/"+ baseName + "_Top." + suffix;
    QString fileNameRight =filePath+ "/"+baseName + "_Right." + suffix;

    if (fileName.isEmpty()) {
        WrongWidget("输入路径错误!");
        return ;
    }
    QString selectedFilter = QFileInfo(fileName).suffix();
    std::string format=selectedFilter.toStdString();

    {
        vtkSmartPointer<vtkRenderWindow> renderWindow=m_pMainWin->getPWinVtkWidget()->getRenderWindow();
        m_pMainWin->onRightViewClicked();
        renderWindow->Render();
        SaveImage(fileNameRight,format);}

    {
        vtkSmartPointer<vtkRenderWindow> renderWindow=m_pMainWin->getPWinVtkWidget()->getRenderWindow();
        m_pMainWin->onTopViewClicked();
        renderWindow->Render();
        SaveImage(fileNameTop,format);}

    {
        vtkSmartPointer<vtkRenderWindow> renderWindow=m_pMainWin->getPWinVtkWidget()->getRenderWindow();
        m_pMainWin->onFrontViewClicked();
        renderWindow->Render();
        SaveImage(fileNameFront,format);}

    QString logInfo="对比图片保存成功";
    m_pMainWin->getPWinVtkPresetWidget()->setWidget(logInfo);

    lastCreatedImageFileFront=fileNameFront;
    lastCreatedImageFileTop=fileNameTop;
    lastCreatedImageFileRight=fileNameRight;
}

void ToolWidget::setauto(bool Auto)
{
    IsAuto=Auto;
}


void ToolWidget::addToList(CEntity* newEntity){
    newEntity->m_CreateForm = ePreset;
    newEntity->m_pRefCoord = m_pMainWin->m_pcsListMgr->m_pPcsCurrent;
    newEntity->m_pCurCoord = m_pMainWin->m_pcsListMgr->m_pPcsCurrent;
    newEntity->m_pExtCoord = m_pMainWin->m_pcsListMgr->m_pPcsCurrent;
    // newEntity->SetNominal();
    // 加入Entitylist 和 ObjectList

    m_pMainWin->m_EntityListMgr->Add(newEntity);
    m_pMainWin->m_ObjectListMgr->Add(newEntity);

    //加入constructEntityList
    constructEntityList.push_back(newEntity);

    //加入contentItemMap
    m_pMainWin->getpWinFileMgr()->getContentItemMap().insert(newEntity->GetObjectCName()+"  "+newEntity->GetObjectAutoName(), true);


}

void ToolWidget::addToFindList(CEntity* newEntity){
    newEntity->m_CreateForm = ePreset;
    newEntity->m_pRefCoord = m_pMainWin->m_pcsListMgr->m_pPcsCurrent;
    newEntity->m_pCurCoord = m_pMainWin->m_pcsListMgr->m_pPcsCurrent;
    newEntity->m_pExtCoord = m_pMainWin->m_pcsListMgr->m_pPcsCurrent;
    // newEntity->SetNominal();
    // 加入Entitylist 和 ObjectList

    m_pMainWin->m_EntityListMgr->Add(newEntity);
    m_pMainWin->m_ObjectListMgr->Add(newEntity);

    //加入constructEntityList
    identifyEntityList.push_back(newEntity);

    //加入identifyItemMap
    m_pMainWin->getpWinFileMgr()->getIdentifyItemMap().insert(newEntity->GetObjectCName()+"  "+newEntity->GetObjectAutoName(), true);


}

void ToolWidget::onConstructPoint(){
    QVector<CPosition>& positions= m_pMainWin->getChosenListMgr()->getChosenActorAxes();
    PointConstructor constructor;
    CPoint* newPoint;
    bool createPoint=false;

    if(positions.size()!=0){
        createPoint=true;
        for(int i=0;i<positions.size();i++){
            newPoint=constructor.createPoint(positions[i]);
            newPoint->Form="构造";
            CPoint*p=newPoint;
            p->Form="构造";
            newPoint->parent.push_back(p);
            if(newPoint!=nullptr){
                addToList(newPoint);
            }
        }
        positions.clear();
    }
    else{
        auto& entityList = m_pMainWin->m_EntityListMgr->getEntityList();
        newPoint=(CPoint*)constructor.create(entityList);
        positions=constructor.getPositions();
        if(newPoint!=nullptr){
            addToList(newPoint);
            for(int i=1;i<positions.size();i++){
                newPoint=constructor.createPoint(positions[i]);
                addToList(newPoint);
            }
        }
    }
    if(!createPoint&&newPoint==nullptr){
        WrongWidget("没有选中或者识别的点");
        return ;
    }
    m_pMainWin->NotifySubscribe();
}

void ToolWidget::onConstructLine(){
    QVector<CPosition>& positions= m_pMainWin->getChosenListMgr()->getChosenActorAxes();
    LineConstructor constructor;
    CLine* newLine;
    bool createLine=false;
    if(positions.size()==2){
        newLine=constructor.createLine(positions[0],positions[1]);
        addToList(newLine);
        createLine=true;
        positions.clear();
        QVector4D pos(positions[0].x-positions[1].x,positions[0].y-positions[1].y,positions[0].z-positions[1].z,0);
        pos.normalize();
    }
    else{
        auto& entityList = m_pMainWin->m_EntityListMgr->getEntityList();
        newLine=(CLine*)constructor.create(entityList);
        if(newLine!=nullptr){
            addToList(newLine);
        }

    }
    if(!createLine&&newLine==nullptr){
        if(positions.size()==1){
            WrongWidget("识别点的数目不足两个");
        }
        else if(positions.size()>=3){
            WrongWidget("识别点的数目超过两个");
        }else if(newLine==nullptr){
            if(constructor.getWrongInformation()==PointTooMuch){
                WrongWidget("列表选中的点过多");
            }else if(constructor.getWrongInformation()==PointTooLess){
                WrongWidget("列表选中的点过少");
            }else if(constructor.getWrongInformation()==PointTooClose){
                WrongWidget("列表选中的点过近");
            }
        }
        return ;
    }
    m_pMainWin->NotifySubscribe();

}
static void WrongWidget(QString message,QString moreMessage){
    QMessageBox msgBox;
    msgBox.setWindowTitle("错误");
    msgBox.setText(message);
    if(moreMessage!="空"){
        msgBox.setInformativeText(moreMessage);
    }
    msgBox.setIcon(QMessageBox::Critical); // 设置对话框图标为错误
    msgBox.setStandardButtons(QMessageBox::Ok); // 只显示“确定”按钮
    msgBox.exec(); // 显示对话框
}

void ToolWidget::onConstructCircle(){
    auto& entityList = m_pMainWin->m_EntityListMgr->getEntityList();
    CircleConstructor constructor;
    CCircle* newCircle=(CCircle*)constructor.create(entityList);
    if(newCircle==nullptr){
        if(constructor.getWrongInformation()==PointTooMuch){
            WrongWidget("列表选中的点过多");
        }else if(constructor.getWrongInformation()==PointTooLess){
            WrongWidget("列表选中的点过少");
        }else if(constructor.getWrongInformation()==PointTooClose){
            WrongWidget("列表选中的点过近");
        }
        return ;
    }
    addToList(newCircle);
    m_pMainWin->NotifySubscribe();

}
void ToolWidget::onConstructPlane(){
    QVector<CPosition>& positions= m_pMainWin->getChosenListMgr()->getChosenActorAxes();
    PlaneConstructor constructor;
    CPlane* newPlane;
    bool createPlane=false;

    if(positions.size()==3){
        newPlane=constructor.createPlane(positions[0],positions[1],positions[2]);
        addToList(newPlane);
        createPlane=true;
        positions.clear();
    }
    else{
        auto& entityList = m_pMainWin->m_EntityListMgr->getEntityList();
        newPlane=(CPlane*)constructor.create(entityList);
        if(newPlane!=nullptr){
            addToList(newPlane);
        }
    }

    if(!createPlane&&newPlane==nullptr){
        if(positions.size()<3){
            WrongWidget("识别点的数目不足三个");
        }else if(positions.size()>3){
            WrongWidget("识别点的数目多余三个");
        }else{
            if(constructor.getWrongInformation()==PointTooMuch){
                WrongWidget("列表选中的点过多");
            }else if(constructor.getWrongInformation()==PointTooLess){
                WrongWidget("列表选中的点过少");
            }else if(constructor.getWrongInformation()==PointTooClose){
                WrongWidget("列表选中的点过近");
            }
        }

        return ;
    }
    m_pMainWin->NotifySubscribe();
}
void ToolWidget::onConstructRectangle(){
    onConstructPlane();
}

void ToolWidget::onConstructSphere(){

    auto& entityList = m_pMainWin->m_EntityListMgr->getEntityList();
    SphereConstructor constructor;
    CSphere* newSphere=(CSphere*)constructor.create(entityList);
    if(newSphere==nullptr){
        if(constructor.getWrongInformation()==PointTooMuch){
            WrongWidget("列表选中的点过多");
        }else if(constructor.getWrongInformation()==PointTooLess){
            WrongWidget("列表选中的点过少");
        }else if(constructor.getWrongInformation()==PointTooClose){
            WrongWidget("列表选中的点过近");
        }else if(constructor.getWrongInformation()==PointDontMatch){
            WrongWidget("四点无法构成球形");
        }
        return ;
    }
    addToList(newSphere);
    m_pMainWin->NotifySubscribe();

}

void ToolWidget::onConstructCone(){
    auto& entityList = m_pMainWin->m_EntityListMgr->getEntityList();
    ConeConstructor constructor;
    CCone* newCone=(CCone*)constructor.create(entityList);
    if(newCone==nullptr){
        if(constructor.getWrongInformation()==PointTooMuch){
            WrongWidget("列表选中的点过多");
        }else if(constructor.getWrongInformation()==PointTooLess){
            WrongWidget("列表选中的点过少");
        }
        return ;
    }
    addToList(newCone);
    m_pMainWin->NotifySubscribe();

}
void ToolWidget::onConstructCylinder(){
    auto& entityList = m_pMainWin->m_EntityListMgr->getEntityList();
    CylinderConstructor constructor;
    CCylinder* newCylinder=(CCylinder*)constructor.create(entityList);
    if(newCylinder==nullptr){
        if(constructor.getWrongInformation()==PointTooMuch){
            WrongWidget("列表选中的点过多");
        }else if(constructor.getWrongInformation()==PointTooLess){
            WrongWidget("列表选中的点过少");
        }else if(constructor.getWrongInformation()==PointTooClose){
            WrongWidget("列表选中的点过近");
        }else if(constructor.getWrongInformation()==PointDontMatch){
            WrongWidget("四点无法构成圆柱");
        }
        return ;
    }
    addToList(newCylinder);
    m_pMainWin->NotifySubscribe();

}
void ToolWidget::onConstructDistance(){
    auto& entityList = m_pMainWin->m_EntityListMgr->getEntityList();
    DistanceConstructor constructor;
    CDistance* newDistance=(CDistance*)constructor.create(entityList);
    if(newDistance==nullptr){
        WrongWidget("构造距离失败");
        return ;
    }
    addToList(newDistance);
    m_pMainWin->NotifySubscribe();

}


void ToolWidget::onConstructPointCloud(){
    auto& entityList = m_pMainWin->m_EntityListMgr->getEntityList();
    auto cloudptr= m_pMainWin->getpWinFileMgr()->cloudptr;
    QVector<CPointCloud*> pointClouds;

    for(int i=0;i<entityList.size();i++){
        CEntity* entity=entityList[i];
        if(!entity->IsSelected())continue;
        if(entity->GetUniqueType()==enPointCloud){
            CPointCloud* pointCloud=(CPointCloud*)entity;
            pointClouds.append(pointCloud);
        }
    }
    if(cloudptr==nullptr){
        WrongWidget("点云指针为空");
        return ;
    }
    PointCloudConstructor constructor;

    for(int i=0;i<pointClouds.size();i++){
        auto& sourceCloud=pointClouds[i]->m_pointCloud;
        constructor.setSourceCloud(sourceCloud.makeShared());
        CPointCloud* newPointCloud=(CPointCloud*)constructor.create(entityList);
        if(newPointCloud==nullptr){
            WrongWidget("构造点云失败");
            continue;
        }
        newPointCloud->isCut=true;
        newPointCloud->m_strAutoName+="(切割)";
        addToList(newPointCloud);
    }


    m_pMainWin->NotifySubscribe();
}
void ToolWidget::onConstructAngle(){
   auto& entityList = m_pMainWin->m_EntityListMgr->getEntityList();
    AngleConstructor constructor;
    CAngle* newAngle=(CAngle*)constructor.create(entityList);
    if(newAngle==nullptr){
       if(constructor.getWrongInformation()==SourceEntityLess){
            WrongWidget("实体数目过少");
       }else{
           WrongWidget("实体数目过多");
       }
    }else{
        addToList(newAngle);
         m_pMainWin->NotifySubscribe();
    }



}

void ToolWidget:: onFindPlane(){
    //读取选中的点云
    auto& entityList = m_pMainWin->m_EntityListMgr->getEntityList();
    QVector<CPointCloud*> pointClouds;

    //读取选中的点
    QVector<CPosition>& positions= m_pMainWin->getChosenListMgr()->getChosenActorAxes();
    pcl::PointXYZRGB  point;
    if(positions.size()==0)return ;
    point.x=positions[0].x;
    point.y=positions[0].y;
    point.z=positions[0].z;

    // 获取拟合用的点云指针
    auto cloudptr= m_pMainWin->getpWinFileMgr()->cloudptr;
    m_pMainWin->getPWinSetDataWidget()->setPlaneData(point,cloudptr);

    // 如果没有从窗口里选中点云，则从列表中获取，列表中也没有选中则报异常
    if(cloudptr==nullptr){
        for(int i=0;i<entityList.size();i++){
            CEntity* entity=entityList[i];
            if(!entity->IsSelected())continue;
            if(entity->GetUniqueType()==enPointCloud){
                CPointCloud* pointCloud=(CPointCloud*)entity;
                pointClouds.append(pointCloud);
            }
        }
        if(pointClouds.size()<1){
            WrongWidget("选中的点云数目为0");
            return ;
        }else if(pointClouds.size()>1){
            WrongWidget("选中的点云数目大于1");
            return ;
        }
        else
            m_pMainWin->getPWinSetDataWidget()->setPlaneData(point,pointClouds[0]->m_pointCloud.makeShared());
        return ;
    }

    auto planeCloud=m_pMainWin->getPWinSetDataWidget()->getPlaneCloud();
    if(planeCloud==nullptr){
        qDebug()<<"拟合平面生成错误";
        return ;
    }
    auto plane=m_pMainWin->getPWinSetDataWidget()->getPlane();
    if(plane==nullptr){
        return;
    }

    // PointConstructor p_constructor;
    // CPoint *newPoint;
    // newPoint=p_constructor.createPoint(point.x,point.y,point.z);
    // addToFindList(newPoint);

    PlaneConstructor constructor;
    CPlane* newPlane;
    CPosition center;
    center.x=plane->getCenter()[0];
    center.y=plane->getCenter()[1];
    center.z=plane->getCenter()[2];
    QVector4D normal(plane->getNormal().x(),plane->getNormal().y(),plane->getNormal().z(),0);
    QVector4D direction(plane->getLength_Direction().x(),plane->getLength_Direction().y(),plane->getLength_Direction().z(),0);
    newPlane=constructor.createPlane(center,normal,direction,plane->getLength(),plane->getWidth());
    if(newPlane==nullptr){
        qDebug()<<"拟合平面生成错误";
        return ;
    }
    addToFindList(newPlane);

    m_pMainWin->getPWinVtkPresetWidget()->setWidget("拟合平面已完成");

    positions.clear();
    m_pMainWin->NotifySubscribe();

}



void ToolWidget::onFindPoint(){
    FittingPoint *nearPoint=new FittingPoint();

    //读取选中的点云
    auto& entityList = m_pMainWin->m_EntityListMgr->getEntityList();
    QVector<CPointCloud*> pointClouds;
    //读取选中的点
    QVector<CPosition>& positions= m_pMainWin->getChosenListMgr()->getChosenActorAxes();
    pcl::PointXYZRGB  point;
    if(positions.size()==0)return ;
    point.x=positions[0].x;
    point.y=positions[0].y;
    point.z=positions[0].z;

    // 获取拟合用的点云指针
    auto cloudptr= m_pMainWin->getpWinFileMgr()->cloudptr;
    nearPoint->RANSAC(point,cloudptr);

    // 如果没有从窗口里选中点云，则从列表中获取，列表中也没有选中则报异常
    if(cloudptr==nullptr){
        for(int i=0;i<entityList.size();i++){
            CEntity* entity=entityList[i];
            if(!entity->IsSelected())continue;
            if(entity->GetUniqueType()==enPointCloud){
                CPointCloud* pointCloud=(CPointCloud*)entity;
                pointClouds.push_back(pointCloud);
            }
        }
        if(pointClouds.size()<1){
            WrongWidget("选中的点云数目为0");
            return ;
        }else if(pointClouds.size()>1){
            WrongWidget("选中的点云数目大于1");
            return ;
        }
        else
            nearPoint->RANSAC(point,pointClouds[0]->m_pointCloud.makeShared());
        return ;
    }

    // PointConstructor p_constructor;
    // CPoint *Point;
    // Point=p_constructor.createPoint(point.x,point.y,point.z);
    // addToFindList(Point);

    qDebug()<<pointClouds.size()<<"hhhhh";

    PointConstructor constructor;
    CPoint* newPoint;
    CPosition center;
    center.x=nearPoint->getPoint()[0];
    center.y=nearPoint->getPoint()[1];
    center.z=nearPoint->getPoint()[2];
    newPoint=constructor.createPoint(center);
    if(newPoint==nullptr){
        qDebug()<<"找到最近点生成错误";
        return ;
    }
    addToFindList(newPoint);

    m_pMainWin->getPWinVtkPresetWidget()->setWidget("拟合点已完成");

    positions.clear();
    m_pMainWin->NotifySubscribe();
}
void ToolWidget::onFindLine(){
    auto& entityList = m_pMainWin->m_EntityListMgr->getEntityList();
    QVector<CPointCloud*> pointClouds;

    //读取选中的点
    QVector<CPosition>& positions= m_pMainWin->getChosenListMgr()->getChosenActorAxes();
    pcl::PointXYZRGB  point;
    if(positions.size()==0)return ;
    point.x=positions[0].x;
    point.y=positions[0].y;
    point.z=positions[0].z;

    // 获取拟合用的点云指针
    auto cloudptr= m_pMainWin->getpWinFileMgr()->cloudptr;

    // 如果没有从窗口里选中点云，则从列表中获取，列表中也没有选中则报异常
    if(cloudptr==nullptr){
        for(int i=0;i<entityList.size();i++){
            CEntity* entity=entityList[i];
            if(!entity->IsSelected())continue;
            if(entity->GetUniqueType()==enPointCloud){
                CPointCloud* pointCloud=(CPointCloud*)entity;
                pointClouds.append(pointCloud);
            }
        }
        if(pointClouds.size()<1){
            WrongWidget("选中的点云数目为0");
            return ;
        }else if(pointClouds.size()>1){
            WrongWidget("选中的点云数目大于1");
            return ;
        }
        else
            m_pMainWin->getPWinSetDataWidget()->setLineData(point,pointClouds[0]->m_pointCloud.makeShared());
        return ;
    }
    m_pMainWin->getPWinSetDataWidget()->setLineData(point, cloudptr);

    // 生成点云对象并添加到entitylist
    auto lineCloud=m_pMainWin->getPWinSetDataWidget()->getLineCloud();
    if(lineCloud==nullptr){
        qDebug()<<"拟合圆柱生成错误";
        return ;
    }
    auto line=m_pMainWin->getPWinSetDataWidget()->getLine();
    if(line==nullptr){
        return;
    }

    // PointConstructor p_constructor;
    // CPoint *newPoint;
    // newPoint=p_constructor.createPoint(point.x,point.y,point.z);
    // addToFindList(newPoint);

    LineConstructor constructor;
    CLine* newLine;
    CPosition begin,end;
    begin.x=line->getBegin().x();
    begin.y=line->getBegin().y();
    begin.z=line->getBegin().z();
    end.x=line->getEnd().x();
    end.y=line->getEnd().y();
    end.z=line->getEnd().z();
    newLine=constructor.createLine(begin,end);
    if(newLine==nullptr){
        qDebug()<<"拟合圆柱生成错误";
        return ;
    }
    addToFindList(newLine);

    m_pMainWin->getPWinVtkPresetWidget()->setWidget("拟合直线已完成");

    positions.clear();
    m_pMainWin->NotifySubscribe();
}



void ToolWidget::onFindCircle(){
    //读取选中的点云
    auto& entityList = m_pMainWin->m_EntityListMgr->getEntityList();
    QVector<CPointCloud*> pointClouds;

    //读取选中的点
    QVector<CPosition>& positions= m_pMainWin->getChosenListMgr()->getChosenActorAxes();
    pcl::PointXYZRGB  point;
    if(positions.size()==0)return ;
    point.x=positions.last().x;
    point.y=positions.last().y;
    point.z=positions.last().z;

    // 获取拟合用的点云指针
    auto cloudptr= m_pMainWin->getpWinFileMgr()->cloudptr;

    // 如果没有从窗口里选中点云，则从列表中获取，列表中也没有选中则报异常
    if(cloudptr==nullptr){
        for(int i=0;i<entityList.size();i++){
            CEntity* entity=entityList[i];
            if(!entity->IsSelected())continue;
            if(entity->GetUniqueType()==enPointCloud){
                CPointCloud* pointCloud=(CPointCloud*)entity;
                pointClouds.append(pointCloud);
            }
        }
        if(pointClouds.size()<1){
            WrongWidget("选中的点云数目为0");
            return ;
        }else if(pointClouds.size()>1){
            WrongWidget("选中的点云数目大于1");
            return ;
        }
        else
            m_pMainWin->getPWinSetDataWidget()->setCircleData(point,pointClouds[0]->m_pointCloud.makeShared());
        return ;
    }
    m_pMainWin->getPWinSetDataWidget()->setCircleData(point, cloudptr);

    auto circleCloud=m_pMainWin->getPWinSetDataWidget()->getCircleCloud();
    if(circleCloud==nullptr){
        qDebug()<<"拟合圆生成错误";
        return ;
    }
    auto circle=m_pMainWin->getPWinSetDataWidget()->getCircle();
    if(circle==nullptr){
        return;
    }

    // PointConstructor p_constructor;
    // CPoint *newPoint;
    // newPoint=p_constructor.createPoint(point.x,point.y,point.z);
    // addToFindList(newPoint);

    CircleConstructor constructor;
    CCircle* newCircle;
    CPosition center;
    center.x=circle->getCenter()[0];
    center.y=circle->getCenter()[1];
    center.z=circle->getCenter()[2];
    double radius=circle->getRad();
    QVector4D normal=QVector4D(circle->getNormal().x(),circle->getNormal().y(),circle->getNormal().z(),1);
    newCircle=constructor.createCircle(center,radius,normal);
    if(newCircle==nullptr){
        qDebug()<<"拟合圆生成错误";
        return ;
    }
    addToFindList(newCircle);

    m_pMainWin->getPWinVtkPresetWidget()->setWidget("拟合圆已完成");

    positions.clear();
    m_pMainWin->NotifySubscribe();
}

void ToolWidget::onFindRectangle(){
    //读取选中的点云
    auto& entityList = m_pMainWin->m_EntityListMgr->getEntityList();
    QVector<CPointCloud*> pointClouds;

    //读取选中的点
    QVector<CPosition>& positions= m_pMainWin->getChosenListMgr()->getChosenActorAxes();
    pcl::PointXYZRGB  point;
    if(positions.size()==0)return ;
    point.x=positions[0].x;
    point.y=positions[0].y;
    point.z=positions[0].z;

    // 获取拟合用的点云指针
    auto cloudptr= m_pMainWin->getpWinFileMgr()->cloudptr;
    m_pMainWin->getPWinSetDataWidget()->setPlaneData(point,cloudptr);

    // 如果没有从窗口里选中点云，则从列表中获取，列表中也没有选中则报异常
    if(cloudptr==nullptr){
        for(int i=0;i<entityList.size();i++){
            CEntity* entity=entityList[i];
            if(!entity->IsSelected())continue;
            if(entity->GetUniqueType()==enPointCloud){
                CPointCloud* pointCloud=(CPointCloud*)entity;
                pointClouds.append(pointCloud);
            }
        }
        if(pointClouds.size()<1){
            WrongWidget("选中的点云数目为0");
            return ;
        }else if(pointClouds.size()>1){
            WrongWidget("选中的点云数目大于1");
            return ;
        }
        else
            m_pMainWin->getPWinSetDataWidget()->setPlaneData(point,pointClouds[0]->m_pointCloud.makeShared());
        return ;
    }

    auto planeCloud=m_pMainWin->getPWinSetDataWidget()->getPlaneCloud();
    if(planeCloud==nullptr){
        qDebug()<<"拟合矩形生成错误";
        return ;
    }
    auto plane=m_pMainWin->getPWinSetDataWidget()->getPlane();
    if(plane==nullptr){
        return;
    }

    // PointConstructor p_constructor;
    // CPoint *newPoint;
    // newPoint=p_constructor.createPoint(point.x,point.y,point.z);
    // addToFindList(newPoint);

    RectangleConstructor constructor;
    CPlane* newPlane;
    CPosition center;
    center.x=plane->getCenter()[0];
    center.y=plane->getCenter()[1];
    center.z=plane->getCenter()[2];
    QVector4D normal(plane->getNormal().x(),plane->getNormal().y(),plane->getNormal().z(),0);
    QVector4D direction(plane->getLength_Direction().x(),plane->getLength_Direction().y(),plane->getLength_Direction().z(),0);
    newPlane=constructor.createRectangle(center,normal,direction,plane->getLength(),plane->getWidth());
    if(newPlane==nullptr){
        qDebug()<<"拟合矩形生成错误";
        return ;
    }
    addToFindList(newPlane);

    m_pMainWin->getPWinVtkPresetWidget()->setWidget("拟合矩形已完成");

    positions.clear();
    m_pMainWin->NotifySubscribe();
}
void ToolWidget::onFindCylinder(){
    auto& entityList = m_pMainWin->m_EntityListMgr->getEntityList();
    QVector<CPointCloud*> pointClouds;

    //读取选中的点
    QVector<CPosition>& positions= m_pMainWin->getChosenListMgr()->getChosenActorAxes();
    pcl::PointXYZRGB  point;
    if(positions.size()==0)return ;
    point.x=positions[0].x;
    point.y=positions[0].y;
    point.z=positions[0].z;

    // 获取拟合用的点云指针
    auto cloudptr= m_pMainWin->getpWinFileMgr()->cloudptr;

    // 如果没有从窗口里选中点云，则从列表中获取，列表中也没有选中则报异常
    if(cloudptr==nullptr){
        for(int i=0;i<entityList.size();i++){
            CEntity* entity=entityList[i];
            if(!entity->IsSelected())continue;
            if(entity->GetUniqueType()==enPointCloud){
                CPointCloud* pointCloud=(CPointCloud*)entity;
                pointClouds.append(pointCloud);
            }
        }
        if(pointClouds.size()<1){
            WrongWidget("选中的点云数目为0");
            return ;
        }else if(pointClouds.size()>1){
            WrongWidget("选中的点云数目大于1");
            return ;
        }
        else
            m_pMainWin->getPWinSetDataWidget()->setCylinderData(point,pointClouds[0]->m_pointCloud.makeShared());
        return ;
    }
    m_pMainWin->getPWinSetDataWidget()->setCylinderData(point, cloudptr);

    // 生成点云对象并添加到entitylist
    auto cylinderCloud=m_pMainWin->getPWinSetDataWidget()->getCylinderCloud();
    if(cylinderCloud==nullptr){
        qDebug()<<"拟合圆柱生成错误";
        return ;
    }
    auto cylinder=m_pMainWin->getPWinSetDataWidget()->getCylinder();
    if(cylinder==nullptr){
        return;
    }

    // PointConstructor p_constructor;
    // CPoint *newPoint;
    // newPoint=p_constructor.createPoint(point.x,point.y,point.z);
    // addToFindList(newPoint);

    CylinderConstructor constructor;
    CCylinder* newCylinder;
    CPosition center;
    center.x=cylinder->getCenter().x();
    center.y=cylinder->getCenter().y();
    center.z=cylinder->getCenter().z();
    QVector4D normal(cylinder->getNormal().x(),cylinder->getNormal().y(),cylinder->getNormal().z(),0);
    newCylinder=constructor.createCylinder(center,normal,cylinder->getHeight(),cylinder->getDiameter());
    if(newCylinder==nullptr){
        qDebug()<<"拟合圆柱生成错误";
        return ;
    }
    addToFindList(newCylinder);

    m_pMainWin->getPWinVtkPresetWidget()->setWidget("拟合圆柱已完成");

    positions.clear();
    m_pMainWin->NotifySubscribe();
}
void ToolWidget::onFindCone(){
    //读取选中的点云
    auto& entityList = m_pMainWin->m_EntityListMgr->getEntityList();
    QVector<CPointCloud*> pointClouds;

    //读取选中的点
    QVector<CPosition>& positions= m_pMainWin->getChosenListMgr()->getChosenActorAxes();
    pcl::PointXYZRGB  point;
    if(positions.size()==0)return ;
    point.x=positions.last().x;
    point.y=positions.last().y;
    point.z=positions.last().z;

    // 获取拟合用的点云指针
    auto cloudptr= m_pMainWin->getpWinFileMgr()->cloudptr;

    // 如果没有从窗口里选中点云，则从列表中获取，列表中也没有选中则报异常
    if(cloudptr==nullptr){
        for(int i=0;i<entityList.size();i++){
            CEntity* entity=entityList[i];
            if(!entity->IsSelected())continue;
            if(entity->GetUniqueType()==enPointCloud){
                CPointCloud* pointCloud=(CPointCloud*)entity;
                pointClouds.append(pointCloud);
            }
        }
        if(pointClouds.size()<1){
            WrongWidget("选中的点云数目为0");
            return ;
        }else if(pointClouds.size()>1){
            WrongWidget("选中的点云数目大于1");
            return ;
        }
        else
            m_pMainWin->getPWinSetDataWidget()->setConeData(point,pointClouds[0]->m_pointCloud.makeShared());
        return ;
    }
    m_pMainWin->getPWinSetDataWidget()->setConeData(point, cloudptr);


    auto coneCloud=m_pMainWin->getPWinSetDataWidget()->getConeCloud();
    if(coneCloud==nullptr){
        qDebug()<<"拟合圆锥生成错误";
        return ;
    }
    auto cone=m_pMainWin->getPWinSetDataWidget()->getCone();
    if(cone==nullptr){
        return;
    }

    // PointConstructor p_constructor;
    // CPoint *newPoint;
    // newPoint=p_constructor.createPoint(point.x,point.y,point.z);
    // addToFindList(newPoint);

    ConeConstructor constructor;
    CCone* newCone;
    CPosition center;
    center.x=cone->getTopCenter()[0];
    center.y=cone->getTopCenter()[1];
    center.z=cone->getTopCenter()[2];
    QVector4D normal(cone->getNormal().x(),cone->getNormal().y(),cone->getNormal().z(),0);
    double angle=cone->getAngle();
    double height=cone->getHeight();
    newCone=constructor.createCone(center,-normal,height,height,angle);
    if(newCone==nullptr){
        qDebug()<<"拟合圆锥生成错误";
        return ;
    }
    addToFindList(newCone);

    m_pMainWin->getPWinVtkPresetWidget()->setWidget("拟合圆锥已完成");

    positions.clear();
    m_pMainWin->NotifySubscribe();
}
void ToolWidget::onFindSphere(){
    //读取选中的点云
    auto& entityList = m_pMainWin->m_EntityListMgr->getEntityList();
    QVector<CPointCloud*> pointClouds;

    //读取选中的点
    QVector<CPosition>& positions= m_pMainWin->getChosenListMgr()->getChosenActorAxes();
    pcl::PointXYZRGB  point;
    if(positions.size()==0)return ;
    point.x=positions.last().x;
    point.y=positions.last().y;
    point.z=positions.last().z;

    // 获取拟合用的点云指针
    auto cloudptr= m_pMainWin->getpWinFileMgr()->cloudptr;

    // 如果没有从窗口里选中点云，则从列表中获取，列表中也没有选中则报异常
    if(cloudptr==nullptr){
        for(int i=0;i<entityList.size();i++){
            CEntity* entity=entityList[i];
            if(!entity->IsSelected())continue;
            if(entity->GetUniqueType()==enPointCloud){
                CPointCloud* pointCloud=(CPointCloud*)entity;
                pointClouds.append(pointCloud);
            }
        }
        if(pointClouds.size()<1){
            WrongWidget("选中的点云数目为0");
            return ;
        }else if(pointClouds.size()>1){
            WrongWidget("选中的点云数目大于1");
            return ;
        }
        else
            m_pMainWin->getPWinSetDataWidget()->setSphereData(point,pointClouds[0]->m_pointCloud.makeShared());
        return ;
    }
    m_pMainWin->getPWinSetDataWidget()->setSphereData(point, cloudptr);

    auto sphereCloud=m_pMainWin->getPWinSetDataWidget()->getSphereCloud();
    if(sphereCloud==nullptr){
        qDebug()<<"拟合球生成错误";
        return ;
    }
    auto sphere=m_pMainWin->getPWinSetDataWidget()->getSphere();
    if(sphere==nullptr){
        return;
    }

    // PointConstructor p_constructor;
    // CPoint *newPoint;
    // newPoint=p_constructor.createPoint(point.x,point.y,point.z);
    // addToFindList(newPoint);

    SphereConstructor constructor;
    CSphere* newSphere;
    CPosition center;
    center.x=sphere->getCenter()[0];
    center.y=sphere->getCenter()[1];
    center.z=sphere->getCenter()[2];
    double radius=sphere->getRad();
    newSphere=constructor.createSphere(center,radius);
    newSphere->setCurrentId();
    if(newSphere==nullptr){
        qDebug()<<"拟合球生成错误";
        return ;
    }
    addToFindList(newSphere);

    m_pMainWin->getPWinVtkPresetWidget()->setWidget("拟合球已完成");

    positions.clear();
    m_pMainWin->NotifySubscribe();
}
void ToolWidget::updateele(){
}
void ToolWidget::NotifySubscribe(){
}


UniqueToolBar& ToolWidget:: getSave(){
    return m_save;
};
UniqueToolBar& ToolWidget:: getFind(){
    return m_find;
};
UniqueToolBar& ToolWidget:: getCoord(){
    return m_coord;
};
UniqueToolBar& ToolWidget:: getViewAngle(){
    return m_viewAngle;
};
UniqueToolBar& ToolWidget:: getConstruct(){
    return m_construct;
};
ToolBarGathter& ToolWidget::getToolBarGather(){
    return m_toolBarGather;
};

int getImagePaths(const QString& directory, QStringList &iconPaths, QStringList &iconNames) {
    QDir dir(directory);
    QFileInfoList fileInfoList = dir.entryInfoList(QDir::Files | QDir::NoSymLinks , QDir::Name);
    int imageCount = 0;

    for (const QFileInfo &fileInfo : fileInfoList) {
        if (fileInfo.suffix() == "jpg" || fileInfo.suffix() == "jpeg" || fileInfo.suffix() == "png" || fileInfo.suffix() == "gif") {
            QString path = fileInfo.absoluteFilePath();
            QString name = fileInfo.fileName();
            iconPaths.append(path);
            iconNames.append(name.left(name.lastIndexOf('.')));

            imageCount++;
        }
    }

    return imageCount;
}

QVector<CEntity*>& ToolWidget::getConstructEntityList(){
    return constructEntityList;
}

QVector<CEntity*>& ToolWidget::getIdentifyEntityList(){
    return identifyEntityList;
}


QVector<QString>& ToolWidget::getImagePaths(){
    return imagePaths;
}
QString ToolWidget::getTimeString(){
    QDateTime currentDateTime = QDateTime::currentDateTime();

    // 获取年、月、日、小时、分钟和秒
    int year = currentDateTime.date().year();
    int month = currentDateTime.date().month();
    int day = currentDateTime.date().day();
    int hour = currentDateTime.time().hour();
    int minute = currentDateTime.time().minute();
    int second = currentDateTime.time().second();

    // 将年、月、日、小时、分钟和秒连接成一个字符串
    QString dateTimeString = QString("%1%2%3%4%5%6")
                                 .arg(year, 4, 10, QChar('0'))
                                 .arg(month, 2, 10, QChar('0'))
                                 .arg(day, 2, 10, QChar('0'))
                                 .arg(hour, 2, 10, QChar('0'))
                                 .arg(minute, 2, 10, QChar('0'))
                                 .arg(second, 2, 10, QChar('0'));
    return dateTimeString;
}

void ToolWidget::SaveImage(QString Path,std::string format){
    vtkSmartPointer<vtkRenderWindow> renderWindow=m_pMainWin->getPWinVtkWidget()->getRenderWindow();
    vtkNew<vtkWindowToImageFilter> windowToImageFilter;

    renderWindow->Render();

    vtkSmartPointer<vtkImageWriter> writer;
    //确认格式
    if (format == "png") {
        writer = vtkSmartPointer<vtkPNGWriter>::New();
    } else if (format == "jpg" || format == "jpeg") {
        writer = vtkSmartPointer<vtkJPEGWriter>::New();
    } else if (format == "tiff" || format == "tif") {
        writer = vtkSmartPointer<vtkTIFFWriter>::New();
    } else if (format == "bmp") {
        writer = vtkSmartPointer<vtkBMPWriter>::New();
    } else {
        std::cerr << "Unsupported format: " << format << std::endl;
        return;
    }

    renderWindow->Render();
    //得到截图

    windowToImageFilter->SetInput(renderWindow);
    windowToImageFilter->SetScale(1);// 缩放因子，可以根据需要调整
    windowToImageFilter->SetInputBufferTypeToRGBA();//RGBA缓冲
    windowToImageFilter->ReadFrontBufferOff();//读取
    windowToImageFilter->Update();


    writer->SetFileName(Path.toStdString().c_str());
    writer->SetInputConnection(windowToImageFilter->GetOutputPort());
    writer->Write();
}
QString ToolWidget::getCompareImagePath(){
    return CompareImagePath;
}


QString  ToolWidget::getParentPath(int step){
    // 获取上一级目录路径
    QString currentPath = QCoreApplication::applicationDirPath();

    QDir dir(currentPath);

    for(int i=0;i<step;i++){

         dir.cdUp();
    }

    QString parentPath = dir.absolutePath();

    return parentPath;

}

void ToolWidget::createFolder(QString path){
    QDir dir;
    if (dir.mkpath(path)) {

        qDebug() << "文件夹创建成功:" << path;
    } else {

        qDebug() << "文件夹创建失败:" << path;
    }
}


QString ToolWidget::getOutputPath(QString kind){
    QString path=getParentPath(2);
    return path+"/输出/"+kind;

}
QDataStream& ToolWidget::serializeEntityList(QDataStream& out, const QVector<CEntity*>& entityList){
    out << static_cast<int>(entityList.size());
    qDebug()<<"out size:"<<static_cast<int>(entityList.size());
    for (const auto& entity : entityList) {
        const CObject* en = dynamic_cast<const CObject*>(entity);
        en->CObject::serialize(out);
    }
    return out;
}

QDataStream& ToolWidget::deserializeEntityList(QDataStream& in, QVector<CEntity*>& entityList){
    int size=0;
    in >> size;
    qDebug()<<"in size:"<<size;
    entityList.clear();
    for(int i=0;i<size;i++){
        CEntity* obj=new CEntity();
        obj->CObject::deserialize(in);
        entityList.append(obj);
    }
    return in;
}
