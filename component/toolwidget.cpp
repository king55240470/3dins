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
#include <QtPrintSupport>
#include <QApplication>
#include <QPdfWriter>
#include <QPainter>
#include <QPageSize>
//word保存
#include <QProcess>
#include <QFileInfo>
//Excelbaocun
//#include <QtXlsx>
//图片保存
#include <QDateTime>


#include <QGuiApplication>
#include <QPdfWriter>
#include <QPainter>
#include <QImage>
#include <QString>
#include <QFont>
#include <QFontDatabase>
#include <QList>
#include <QDateTime>
#include <QPixmap>
#include <QBrush>
#include <QPageSize>
#include <QPen>
#include <cstdlib>


#include <pcl/io/ply_io.h>  // 用于保存 .ply 文件
#include <pcl/io/pcd_io.h>  // 用于保存 .pcd 文件
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// bool convertPdfToWord(const QString &pdfFilePath, const QString &wordFilePath) {
//     QProcess process;
//     QStringList arguments;
//     arguments << "--headless" << "--convert-to" << "docx" << "--outdir"
//               << QFileInfo(wordFilePath).absolutePath() << pdfFilePath;

//     process.start("libreoffice", arguments);
//     process.waitForFinished();

//     return process.exitStatus() == QProcess::NormalExit;
// }



int getImagePaths(const QString& directory, QStringList &iconPaths, QStringList &iconNames);

ToolWidget::ToolWidget(QWidget *parent)
    : QWidget(parent) {


    QVBoxLayout *layout = new QVBoxLayout(this);

    m_pMainWin =(MainWindow*)parent;


    InitOutputFolder();

    m_savePdf=false;
    Action_Checked=nullptr;
    Is_FindPoint_Cheked=false;


    resize(400,250);

    m_nSaveActionNum=5;
    m_nConstructActionNum=10;
    m_nFindActionNum=8;
    m_nCoordActionNum=3;
    m_nViewAngleActionNum=4;

    //静态保存图片路径和名称


    //保存图标路径
    save_action_iconpath_list_<<":/component/save/excel.png"<< ":/component/save/pdf.jpg"<< ":/component/save/txt.jpg"<< ":/component/save/word.jpg"<<":/component/save/image.png"<<":/component/save/pointcloud.png";
    construct_action_iconpath_list_<<":/component/construct/point_.png"<<":/component/construct/line_.png"<<":/component/construct/circle_.png"<<   ":/component/construct/plane_.png"<<  ":/component/construct/rectangle_.png"<<":/component/construct/cylinder_.png"<< ":/component/construct/cone_.png"<< ":/component/construct/sphere_.png"<<":/component/construct/distance.png"<<":/component/construct/pointCloud.png"<<":/component/construct/angle.png";
    find_action_iconpath_list_<<":/component/construct/point.jpg"<<":/component/construct/line.jpg"<<":/component/construct/circle.jpg"<<   ":/component/construct/plan.jpg"<<  ":/component/construct/rectangle.jpg"<<":/component/construct/cylinder.jpg"<< ":/component/construct/cone.jpg"<< ":/component/construct/sphere.jpg";
    coord_action_iconpath_list_<<":/component/coord/create.png"<<  ":/component/coord/spin.jpg"<<":/component/coord/save.png";
    view_angle_action_iconpath_list_<<":/component/viewangle/front.png"<<":/component/viewangle/up.png"<<":/component/viewangle/right.png"<<":/component/viewangle/isometric.png";




   //设置图标名称
    save_action_name_list_<<"excel"<< "pdf"<< "txt"<< "word"<<"image"<<"pointcloud";
    construct_action_name_list_<<"构造点"<<"构造线"<<"构造圆"<<"构造平面"<<"构造矩形"<<"构造圆柱"<<"构造圆锥"<<"构造球形"<<"构造距离"<<"构造点云"<<"构造角度";
    find_action_name_list_<<"识别点"<<"识别线"<<"识别圆"<<"识别平面"<<"识别矩形"<<"识别圆柱"<<"识别圆锥"<<"识别球形";;
    coord_action_name_list_<<"创建坐标系"<<"旋转坐标系"<<"保存坐标系";
    view_angle_action_name_list_<<"主视角"<<"俯视角"<<"侧视角"<<"立体视角";

   //更新图标数目
    m_nSaveActionNum=save_action_name_list_.count();
    m_nConstructActionNum=construct_action_name_list_.count();
    m_nFindActionNum=find_action_name_list_.count();
    m_nCoordActionNum=coord_action_name_list_.count();
    m_nViewAngleActionNum=view_angle_action_name_list_.count();

    //创建图标
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

    //设置工具栏格式
    for(int i=0;i<m_nToolbarNum;i++){

        toolBars[i]=new QToolBar(this);
        toolBars[i]->setIconSize(QSize(iconsize,iconsize));
        toolBars[i]->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        toolBars[i]->setStyleSheet(
            "QToolButton {"
            "    border: 2px solid lightgray;" // 浅灰色边框
            "    padding: 0 px;" // 内部填充
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
        action->setCheckable(true); // 使QAction可切换
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
    QString ParentPath="D:";
    CompareImagePath= ParentPath+"/点云对比图像/screenshot"+"/";

    // createFolder(ParentPath+"/点云对比图像/screenshot");
    createFolder(getOutputPath("pcd"));
    createFolder(getOutputPath("ply"));
    createFolder(getOutputPath("word"));
    createFolder(getOutputPath("pdf"));
    createFolder(getOutputPath("image"));
    createFolder(getOutputPath("excel"));
    createFolder(getOutputPath("txt"));
    // createFolder(ParentPath+"/output/pdf");
    // createFolder(ParentPath+"/output/image");
    // createFolder(ParentPath+"/output/excel");
    // createFolder(ParentPath+"/output/txt");
    m_pMainWin->getPWinVtkPresetWidget()->setWidget("已创建文件夹"+getOutputPath(""));

}
void ToolWidget::clearToolWidget(){

    //清空
    for(int i=0;i<m_nToolbarNum;i++){
        clearToolBar(toolBars[i]);
    }

}
void ToolWidget::createToolWidget(){
    //重构工具栏
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
    connect(find_actions_[find_action_name_list_.indexOf("识别点")],&QAction::triggered,this,[&](){

        QAction *action = qobject_cast<QAction *>(sender());
        if (action->isChecked()) {
            //标注选中状态
            Is_FindPoint_Cheked=true;
            action->setIcon(QIcon(":/component/find/point_.png"));
            action->setToolTip("已选中");
            QString log="开始识别点, 记录选中点的坐标";
            m_pMainWin->getPWinVtkPresetWidget()->setWidget(log);
        } else {
            Is_FindPoint_Cheked=false;
            action->setIcon(QIcon(":/component/find/point.jpg"));
            action->setToolTip("识别点");

            QString log="结束识别点";
            m_pMainWin->getPWinVtkPresetWidget()->setWidget(log);
        }});


    connect(find_actions_[find_action_name_list_.indexOf("识别线")],&QAction::triggered,this,[&](){
        QAction *action = qobject_cast<QAction *>(sender());
        if (action->isChecked()) {
            //若已有选中状态的QAction，则使该QAction退出选中状态
            if(Action_Checked!=nullptr&&Action_Checked!=action){
                //Action_Checked->setChecked(false);
                Action_Checked->trigger();
            }
            Action_Checked=(ToolAction*)action;
            action->setIcon(QIcon(":/component/find/line_.png"));
            action->setToolTip("已选中");

            QString log="开始识别线";
            m_pMainWin->getPWinVtkPresetWidget()->setWidget(log);



        } else {
            //若记录的是当前的QAction，则将记录置空
            if(Action_Checked==action){
                Action_Checked=nullptr;
            }
            action->setIcon(QIcon(":/component/find/line.jpg"));
            action->setToolTip("识别线");

            QString log="结束识别线";
            m_pMainWin->getPWinVtkPresetWidget()->setWidget(log);
        }});
    // connect(find_actions_[find_action_name_list_.indexOf("识别圆")],&QAction::triggered,this,&   ToolWidget::onFindCircle);


    connect(find_actions_[find_action_name_list_.indexOf("识别圆")],&QAction::triggered,this,[&](){
    QAction *action = qobject_cast<QAction *>(sender());
    if (action->isChecked()) {
        if(Action_Checked!=nullptr&&Action_Checked!=action){
            //Action_Checked->setChecked(false);
            Action_Checked->trigger();

        }
          Action_Checked=(ToolAction*)action;
        action->setIcon(QIcon(":/component/find/circle_.jpg"));
        action->setToolTip("已选中");

        QString log="开始识别圆";
        m_pMainWin->getPWinVtkPresetWidget()->setWidget(log);
    } else {
        if(Action_Checked==action){
            Action_Checked=nullptr;
        }
        action->setIcon(QIcon(":/component/find/circle.jpg"));
        action->setToolTip("识别圆");

        QString log="结束识别圆";
        m_pMainWin->getPWinVtkPresetWidget()->setWidget(log);
    }});


    connect(find_actions_[find_action_name_list_.indexOf("识别平面")],&QAction::triggered,this,[&](){
        QAction *action = qobject_cast<QAction *>(sender());
        if (action->isChecked()) {
            if(Action_Checked!=nullptr&&Action_Checked!=action){
               // Action_Checked->setChecked(false);
                Action_Checked->trigger();

            }
            Action_Checked=(ToolAction*)action;
            action->setIcon(QIcon(":/component/find/plane_.png"));
            action->setToolTip("已选中");

            QString log="开始识别平面";
            m_pMainWin->getPWinVtkPresetWidget()->setWidget(log);
        } else {
            if(Action_Checked==action){
                Action_Checked=nullptr;
            }
            action->setIcon(QIcon(":/component/find/plan.jpg"));
            action->setToolTip("识别平面");

            QString log="结束识别平面";
            m_pMainWin->getPWinVtkPresetWidget()->setWidget(log);
        }});


    connect(find_actions_[find_action_name_list_.indexOf("识别矩形")],&QAction::triggered,this,[&](){
        QAction *action = qobject_cast<QAction *>(sender());
        if (action->isChecked()) {
            if(Action_Checked!=nullptr&&Action_Checked!=action){
               // Action_Checked->setChecked(false);
                Action_Checked->trigger();

            }
             Action_Checked=(ToolAction*)action;
            action->setIcon(QIcon(":/component/find/rectangle_.png"));
            action->setToolTip("已选中");

            QString log="开始识别矩形";
            m_pMainWin->getPWinVtkPresetWidget()->setWidget(log);
        } else {
            if(Action_Checked==action){
                Action_Checked=nullptr;
            }
            action->setIcon(QIcon(":/component/find/rectangle.jpg"));
            action->setToolTip("识别矩形");

            QString log="结束识别矩形";
            m_pMainWin->getPWinVtkPresetWidget()->setWidget(log);
        }});


    connect(find_actions_[find_action_name_list_.indexOf("识别圆柱")],&QAction::triggered,this,[&](){
        QAction *action = qobject_cast<QAction *>(sender());
        if (action->isChecked()) {
            if(Action_Checked!=nullptr&&Action_Checked!=action){
                //Action_Checked->setChecked(false);
                Action_Checked->trigger();
            }
            Action_Checked=(ToolAction*)action;
            action->setIcon(QIcon(":/component/find/cylinder_.jpg"));
            action->setToolTip("已选中");

            QString log="开始识别圆柱";
            m_pMainWin->getPWinVtkPresetWidget()->setWidget(log);
        } else {
            if(Action_Checked==action){
                Action_Checked=nullptr;
            }
            action->setIcon(QIcon(":/component/find/cylinder.jpg"));
            action->setToolTip("识别圆柱");

            QString log="结束识别圆柱";
            m_pMainWin->getPWinVtkPresetWidget()->setWidget(log);
        }});


    connect(find_actions_[find_action_name_list_.indexOf("识别圆锥")],&QAction::triggered,this,[&](){
        QAction *action = qobject_cast<QAction *>(sender());
        if (action->isChecked()) {
            if(Action_Checked!=nullptr&&Action_Checked!=action){
                //Action_Checked->setChecked(false);
                Action_Checked->trigger();

            }
            Action_Checked=(ToolAction*)action;
            action->setIcon(QIcon(":/component/find/cone_.jpg"));
            action->setToolTip("已选中");

            QString log="开始识别圆锥";
            m_pMainWin->getPWinVtkPresetWidget()->setWidget(log);
        } else {
            if(Action_Checked==action){
                Action_Checked=nullptr;
            }
            action->setIcon(QIcon(":/component/find/cone.jpg"));
            action->setToolTip("识别圆锥");

            QString log="结束识别圆锥";
            m_pMainWin->getPWinVtkPresetWidget()->setWidget(log);
        }});


    connect(find_actions_[find_action_name_list_.indexOf("识别球形")],&QAction::triggered,this,[&](){
        QAction *action = qobject_cast<QAction *>(sender());
        if (action->isChecked()) {
            if(Action_Checked!=nullptr&&Action_Checked!=action){
               // Action_Checked->setChecked(false);
                Action_Checked->trigger();

            }
            Action_Checked=(ToolAction*)action;
            action->setIcon(QIcon(":/component/find/sphere_.png"));
            action->setToolTip("已选中");

            QString log="开始识别球形";
            m_pMainWin->getPWinVtkPresetWidget()->setWidget(log);
        } else {
            if(Action_Checked==action){
                Action_Checked=nullptr;
            }
            action->setIcon(QIcon(":/component/find/sphere.jpg"));
            action->setToolTip("识别球形");
            QString log="结束识别球形";
            m_pMainWin->getPWinVtkPresetWidget()->setWidget(log);
        }});



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
    //connect(save_actions_[save_action_name_list_.indexOf("excel")],&QAction::triggered,this,&  ToolWidget::onSaveExcel);
     //connect(save_actions_[save_action_name_list_.indexOf("word")],&QAction::triggered,this,&  ToolWidget::onSaveWord);
    // connect(save_actions_[save_action_name_list_.indexOf("txt")],&QAction::triggered,this,&  ToolWidget::onSaveTxt);
    // connect(save_actions_[save_action_name_list_.indexOf("pdf")],&QAction::triggered,this,&  ToolWidget::onSavePdf);
    // connect(save_actions_[save_action_name_list_.indexOf("image")],&QAction::triggered,this,&  ToolWidget::onSaveImage);
    connect(save_actions_[save_action_name_list_.indexOf("pointcloud")],&QAction::triggered,this,&  ToolWidget::onSavePointCloud);
    //打开
    connect(save_actions_[save_action_name_list_.indexOf("excel")], &QAction::triggered, this, &ToolWidget::onOpenExcel);
    connect(save_actions_[save_action_name_list_.indexOf("word")], &QAction::triggered, this, &ToolWidget::onOpenWord);
    connect(save_actions_[save_action_name_list_.indexOf("txt")], &QAction::triggered, this, &ToolWidget::onOpenTxt);
    connect(save_actions_[save_action_name_list_.indexOf("pdf")], &QAction::triggered, this, &ToolWidget::onOpenPdf);
    connect(save_actions_[save_action_name_list_.indexOf("image")], &QAction::triggered, this, &ToolWidget::onOpenImage);

    // connect(save_actions_[save_action_name_list_.indexOf("txt")],&QAction::triggered,this,&  ToolWidget::onSaveTxt);


    //坐标系
    connect(coord_actions_[coord_action_name_list_.indexOf("创建坐标系")],&QAction::triggered,this,[&](){

        m_pMainWin->on2dCoordOriginAuto(); //创建临时坐标系
    });
    connect(coord_actions_[coord_action_name_list_.indexOf("旋转坐标系")],&QAction::triggered,this,[&](){

        m_pMainWin->on2dCoordSetRightX(); // x轴摆正
    });
    connect(coord_actions_[coord_action_name_list_.indexOf("保存坐标系")],&QAction::triggered,this,[&](bool){

        m_pMainWin->on2dCoordSave();
    });
    //视角
    connect(view_angle_actions_[view_angle_action_name_list_.indexOf("主视角")],&QAction::triggered,this,[&](){

        m_pMainWin->onFrontViewClicked();
    });
    connect(view_angle_actions_[view_angle_action_name_list_.indexOf("俯视角")],&QAction::triggered,this, [&](){

        m_pMainWin->onTopViewClicked();
    });
    connect(view_angle_actions_[view_angle_action_name_list_.indexOf("侧视角")],&QAction::triggered,[&](){

        m_pMainWin->onRightViewClicked();
    });
    connect(view_angle_actions_[view_angle_action_name_list_.indexOf("立体视角")],&QAction::triggered,[&](){

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
        if(entity->GetUniqueType()==enDistance){
            CDistance* Distance=(CDistance*) entity;
            inList<<"距离";

            inList<<Distance->m_strAutoName;
            inList<<QString::number(abs(Distance->getdistance()),'f',6);
            inList<<QString::number(Distance->getUptolerance(),'f',6);
            inList<<QString::number(Distance->getUndertolerance(),'f',6);
            if (Distance->judge())
                inList<<"合格";
            else
                inList<<"不合格";
        }
        else if (entity->GetUniqueType()==enAngle){
            CAngle* Angle=(CAngle*)entity;
            inList<<"角度";
            inList<<Angle->m_strAutoName;
            inList<<QString::number(abs(Angle->getAngleValue()),'f',6);
            inList<<QString::number(Angle->getUptolerance(),'f',6);
            inList<<QString::number(Angle->getUndertolerance(),'f',6);
            if (Angle->judge())
                inList<<"合格";
            else
                inList<<"不合格";
        }
        if(inList.size())
        dataAll.append(inList);
    }
}
void   ExtractData(QVector<CEntity *>& entitylist,QList<QList<QString>>& dataAll,QList<QList<QString>>& data_accepted,QList<QList<QString>>& data_not_accepted){
    for (int i = 0; i < entitylist.size(); i++) {
        QList<QString> inList;
        CEntity* entity=entitylist[i];
        if(entity->GetUniqueType()==enDistance){
            CDistance* Distance=(CDistance*) entity;
            inList<<"距离";
            inList<<Distance->m_strAutoName;
            inList<<QString::number(abs(Distance->getdistance()),'f',6);
            inList<<QString::number(Distance->getUptolerance(),'f',6);
            inList<<QString::number(Distance->getUndertolerance(),'f',6);
            if (Distance->judge()){
                inList<<"合格";
            data_accepted.append(inList);
            }
            else{
                inList<<"不合格";
            data_not_accepted.append(inList);
            }
        }
        else if (entity->GetUniqueType()==enAngle){
            CAngle* Angle=(CAngle*)entity;
            inList<<"角度";
            inList<<Angle->m_strAutoName;
            inList<<QString::number(abs(Angle->getAngleValue()),'f',6);
            inList<<QString::number(Angle->getUptolerance(),'f',6);
            inList<<QString::number(Angle->getUndertolerance(),'f',6);
            if (Angle->judge()){
                inList<<"合格";
                data_accepted.append(inList);
            }
            else{
                inList<<"不合格";
                data_not_accepted.append(inList);
            }
        }
        if(inList.size()){

            dataAll.append(inList);

        }
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

    QString logInfo1="Pdf开始保存";
    m_pMainWin->getPWinVtkPresetWidget()->setWidget(logInfo1);

   QString path= getOutputPath("pdf");
   QString name=getTimeString();
   QString filePath=path+"/"+name+".pdf";



   // // 插入纯文本数据
   auto& entitylist = m_pMainWin->m_EntityListMgr->getEntityList();
   QList<QList<QString>> dataAll;
   QList<QList<QString>> dataAccepted;
   QList<QList<QString>>  dataNotAccepted;

   int count_accpted=0,count_not_accepted=0;

   ExtractData(entitylist, dataAll, dataAccepted,dataNotAccepted);

   count_accpted=dataAccepted.size();
   count_not_accepted=dataNotAccepted.size();

   QPdfWriter pdf(filePath);

   pdf.setPageSize(QPageSize(QPageSize::A4));
   pdf.setResolution(300);
   pdf.setTitle("专业测量报告");

   QPainter painter;
   if (painter.begin(&pdf)==false){
       QString logInfo="Pdf打开失败,请关闭Pdf重试";
       m_pMainWin->getPWinVtkPresetWidget()->setWidget(logInfo);
       return ;
   }
   painter.setRenderHint(QPainter::Antialiasing);



   int PDFheight= pdf.height();
   int PDFwidth=pdf.width();


   // 设置字体
   QFont font;
   font.setFamily("Arial");
   font.setPointSize(12);
   painter.setFont(font);
   int yDistance=60;
   int xDistance=300;

   // 封面页
   {
       painter.translate(50, 50); // 左上角偏移
       painter.setPen(Qt::black);

       int PDFwidth = pdf.width();
       int yDistance = 240;

       // 设置标题
       QFont titleFont("Arial", 35, QFont::Bold);
       painter.setFont(titleFont);
       QString title = "工业测量报告";
       QFontMetrics titleMetrics(titleFont);
       int titleWidth = titleMetrics.horizontalAdvance(title)*3;

       // 绘制图片
       QImage image(":/style/ruler.png");
       QImage scaledImage = image.scaled(200, 100);
       int imageWidth = scaledImage.width();
       int imageHeight = scaledImage.height();

       // 计算标题和图片的总宽度
       int totalWidth = titleWidth + imageWidth + 20; // 20是标题和图片之间的间距
       int startX = (PDFwidth - totalWidth) / 2;
       int startY=500;

       painter.drawText(startX, startY, title);
       painter.drawImage(startX + titleWidth + 20, startY-imageHeight/2 , scaledImage);
       int moreY=500;
       // 设置副标题
       QFont subtitleFont("Arial", 20);
       painter.setFont(subtitleFont);
       QFontMetrics subtitleMetrics(subtitleFont);
       QString project = "测量项目: 3D 测量系统";
       QString date = "测量日期: " + QDateTime::currentDateTime().toString("yyyy-MM-dd");
       QString unit = "测量单位: 三维工业测量软件开发组";
       QString Description1="";

       int projectWidth = subtitleMetrics.horizontalAdvance(project)*3;
       int dateWidth = subtitleMetrics.horizontalAdvance(date)*3;
       int unitWidth = subtitleMetrics.horizontalAdvance(unit)*3;

       painter.drawText((PDFwidth - unitWidth) / 2, yDistance+startY+moreY, project);
       painter.drawText((PDFwidth - unitWidth) / 2, 2 * yDistance+startY+moreY, date);
       painter.drawText((PDFwidth - unitWidth) / 2, 3 * yDistance+startY+moreY, unit);
       int lessY=300;
       // 为了使内容美观，增加一个水平线
       painter.drawLine(0, PDFheight-yDistance/2*4-lessY, PDFwidth - 100, PDFheight-yDistance/2*4-lessY);

       // 添加一些描述性文本
       QFont contentFont("Arial", 16);
       painter.setFont(contentFont);
       QFontMetrics contentMetrics(contentFont);
       QString description1 = "本报告详细记录了使用3D测量系统进行的测量项目。";
       QString description2 = "测量日期为" + QDateTime::currentDateTime().toString("yyyy-MM-dd") + "，";
       QString description3 = "由三维工业测量软件开发组负责执行和分析。";

       int description1Width = contentMetrics.horizontalAdvance(description1)*3;
       int description2Width = contentMetrics.horizontalAdvance(description2)*3;
       int description3Width = contentMetrics.horizontalAdvance(description3)*3;

       painter.drawText((PDFwidth - description1Width) / 2, PDFheight-yDistance/2*3-lessY, description1);
       painter.drawText((PDFwidth - description2Width) / 2, PDFheight-yDistance/2*2-lessY, description2);
       painter.drawText((PDFwidth - description3Width) / 2, PDFheight-yDistance/2-lessY, description3);

       pdf.newPage();
   }

   // 目录
   {
       QFont subtitleFont("Arial", 20);
       painter.setFont(subtitleFont);
       QFontMetrics subtitleMetrics(subtitleFont);
       QString description="目录";
       int descrtptionWidth=subtitleMetrics.horizontalAdvance(description)*3;
       painter.resetTransform();
       painter.translate(50, 50);
       painter.setFont(QFont("Arial", 25, QFont::Bold));

       int startY=500;
       int startX=300;
       painter.drawText((PDFwidth- descrtptionWidth)/2, startY, "目录");
       painter.setFont(QFont("Arial", 20));
       int moreY=500;
       int yDistance=240;

       painter.drawText(startX, startY+moreY, "1. 测量数据概览");
       painter.drawText(startX,startY+moreY+ yDistance, "2. 数据分析");
       painter.drawText(startX,startY+moreY+ yDistance*2, "3. 测量结果图");
       painter.drawText(startX,startY+moreY + yDistance*3, "4. 结论与建议");
       painter.drawText(startX,startY+moreY + yDistance*4, "5. 附录");

       pdf.newPage();
   }

   // 测量数据概览
   {
       painter.resetTransform();
       painter.translate(50, 50);
       painter.setFont(QFont("Arial", 14, QFont::Bold));
       painter.drawText(0, 0, "1. 测量数据概览");

       int tableX = 0;
       int tableY = 40;


       // 绘制表格边框
       painter.setPen(Qt::black);


       // 绘制表头
       painter.setFont(QFont("Arial", 12, QFont::Bold));
       painter.drawText(tableX + 20, tableY + 30, "类型");
       painter.drawText(tableX + xDistance+ 20, tableY + 30, "名称");
       painter.drawText(tableX + xDistance*2+ 20, tableY + 30, "数值");
       painter.drawText(tableX + xDistance*3+ 20, tableY + 30, "上公差");
       painter.drawText(tableX + xDistance*4+ 20, tableY + 30, "下公差");
       painter.drawText(tableX + xDistance*5+ 20,tableY + 30,"是否合格");

       painter.setFont(QFont("Arial", 12));
       int yPos = 90;
       bool MoreThanOne=false;
       for (const QList<QString>& rowData : dataAll) {
           if(yPos+yDistance>PDFheight){
               if(MoreThanOne==false)
                   painter.drawRect(tableX, tableY+40,tableX + xDistance*6+40,PDFheight-20);
               else
                   painter.drawRect(tableX, 10,tableX + xDistance*6+40, yPos);

               pdf.newPage();
               painter.drawText(tableX ,  0, "类型");
               painter.drawText(tableX + xDistance, 0, "名称");
               painter.drawText(tableX + xDistance*2,  0, "数值");
               painter.drawText(tableX + xDistance*3, 0, "上公差");
               painter.drawText(tableX + xDistance*4,  0, "下公差");
               painter.drawText(tableX + xDistance*5,  0, "是否合格");
               yPos=30;
               MoreThanOne=true;
           }
           int count=0;
           for (const QString& cellData : rowData) {
                painter.drawText(tableX + 20+count*xDistance, tableY +yPos, cellData);
                count++;
           }

           yPos += yDistance;
       }
       if(MoreThanOne){
           painter.drawRect(tableX, 10,tableX + xDistance*6+40, yPos);
       }else{
           painter.drawRect(tableX, tableY+40,tableX + xDistance*6+40, yPos);
       }

       pdf.newPage();
   }

   // 数据分析
   {

       painter.resetTransform();
       painter.translate(50, 50);
       painter.setFont(QFont("Arial", 14, QFont::Bold));
       painter.drawText(0, 0, "2. 数据分析（不合格参数）");

       int tableX = 0;
       int tableY = 40;


       // 绘制表格边框
       painter.setPen(Qt::black);


       // 绘制表头
       painter.setFont(QFont("Arial", 12, QFont::Bold));
       painter.drawText(tableX + 20, tableY + 30, "类型");
       painter.drawText(tableX + xDistance+ 20, tableY + 30, "名称");
       painter.drawText(tableX + xDistance*2+ 20, tableY + 30, "数值");
       painter.drawText(tableX + xDistance*3+ 20, tableY + 30, "上公差");
       painter.drawText(tableX + xDistance*4+ 20, tableY + 30, "下公差");
       painter.drawText(tableX + xDistance*5+ 20,tableY + 30,"是否合格");

       painter.setFont(QFont("Arial", 12));
       int yPos = 90;
       bool MoreThanOne=false;
       for (const QList<QString>& rowData : dataNotAccepted) {
           if(yPos+yDistance>PDFheight){
               if(MoreThanOne==false)
                   painter.drawRect(tableX, tableY+40,tableX + xDistance*6+40,PDFheight-20);
               else
                   painter.drawRect(tableX, 10,tableX + xDistance*6+40, yPos);

               pdf.newPage();
               painter.drawText(tableX ,  0, "类型");
               painter.drawText(tableX + xDistance, 0, "名称");
               painter.drawText(tableX + xDistance*2,  0, "数值");
               painter.drawText(tableX + xDistance*3, 0, "上公差");
               painter.drawText(tableX + xDistance*4,  0, "下公差");
               painter.drawText(tableX + xDistance*5,  0, "是否合格");
               yPos=30;
               MoreThanOne=true;
           }
           int count=0;
           for (const QString& cellData : rowData) {
               painter.drawText(tableX + 20+count*xDistance, tableY +yPos, cellData);
               count++;
           }

           yPos += yDistance;
       }
       if(MoreThanOne){
           painter.drawRect(tableX, 10,tableX + xDistance*6+40, yPos);
       }else{
           painter.drawRect(tableX, tableY+40,tableX + xDistance*6+40, yPos);
       }

       pdf.newPage();

   }

   // 测量结果图
   {
       painter.resetTransform();
       painter.translate(50, 50);
       painter.setFont(QFont("Arial", 14, QFont::Bold));
       painter.drawText(0, 0, "3. 测量结果图");
       painter.drawText(0, 60, "（1）全局对比");
       int imageX = 0;
       int imageY = 50+30;
       int imageWidth = 1422;
       int imageHeight = 1002;
       int count=1;
       for (const QString &imagePath : imagePaths) {
           QImage image(imagePath);
           if (!image.isNull()) {
               if (imageY + image.height() > pdf.height() - 100) {
                   pdf.newPage();
                   painter.resetTransform();
                   painter.translate(50, 50);
                   painter.setFont(QFont("Arial", 12));
                   imageY = 50;
               }
               painter.drawImage((PDFwidth-imageWidth)/2, imageY, image.scaled(imageWidth, imageHeight));
               painter.drawText((PDFwidth-imageWidth)/2+400, imageY + imageHeight + 50, "图 " + QString::number(count) + ": 全局对比结果图");
               imageY += imageHeight + 60;
               count++;
           }
       }

       pdf.newPage();
   }
   {
       painter.resetTransform();
       painter.translate(50, 50);
       painter.setFont(QFont("Arial", 14, QFont::Bold));
       painter.drawText(0, 0, "（2）局部对比");

       int imageX = 0;
       int imageY = 50;
       int imageWidth = 1422;
       int imageHeight = 1002;
       int count=1;
       for (const QString &imagePath : imagePaths_part) {
           QImage image(imagePath);
           if (!image.isNull()) {
               if (imageY + image.height() > pdf.height() - 100) {
                   pdf.newPage();
                   painter.resetTransform();
                   painter.translate(50, 50);
                   painter.setFont(QFont("Arial", 12));
                   imageY = 50;
               }
               painter.drawImage((PDFwidth-imageWidth)/2, imageY, image.scaled(imageWidth, imageHeight));
               painter.drawText((PDFwidth-imageWidth)/2+400, imageY + imageHeight + 50, "图 " + QString::number(count) + ": 局部对比结果图");
               imageY += imageHeight + 60;
               count++;
           }
       }

       pdf.newPage();
   }

   // 结论与建议
   {
       painter.resetTransform();
       painter.translate(50, 50);
       painter.setFont(QFont("Arial", 14, QFont::Bold));
       painter.drawText(0, 0, "4. 结论与建议");

       painter.setFont(QFont("Arial", 12));
       painter.drawText(0, yDistance, "根据测量数据和分析结果，我们得出以下结论:");
       painter.drawText(0, yDistance*2, "-共检测出"+ QString::number(count_not_accepted)+"个不合格的指标");
       painter.drawText(0, yDistance*3, "-共检测出"+ QString::number(count_accpted)+"个合格的指标");

       pdf.newPage();
   }

   // 附录
   {
       painter.resetTransform();
       painter.translate(50, 50);
       painter.setFont(QFont("Arial", 14, QFont::Bold));
       painter.drawText(0, 0, "5. 附录");

       painter.setFont(QFont("Arial", 12));
       painter.drawText(0, yDistance, "参考文献:");
       painter.drawText(0, yDistance*2, "- 3d工业测量软件开发组测量标准");
       painter.drawText(0, yDistance*3, "- 测量系统用户手册");

   }

   painter.end();

   QString logInfo="Pdf保存成功";
   m_pMainWin->getPWinVtkPresetWidget()->setWidget(logInfo);
   lastCreatedPdfFile=filePath;
   m_savePdf=true;
}





void   ToolWidget::onSaveExcel(){
    QString logInfo1="Excel开始保存";
    m_pMainWin->getPWinVtkPresetWidget()->setWidget(logInfo1);

    QString path= getOutputPath("excel");
    QString name=getTimeString();

    QString filePath=path+"/"+name+".xlsx";
    // QString filePath = QFileDialog::getSaveFileName(nullptr, QString("Save As"), "", QString("Excel(*.xlsx *.xls)"));
    // if (filePath.isEmpty()){
    //     return ;
    // }

    QStringList headers;
    headers << "类型" << "名称" <<"数值"<< "上公差" << "下公差" <<"是否合格" ;
    int col = headers.size();
    QList<QList<QString>> dataAll;
    auto& entitylist = m_pMainWin->m_EntityListMgr->getEntityList();
    //dataAll.append(headers);
    ExtractData(entitylist,dataAll);                              //将数据传入dataAll中

    QAxObject excel("Excel.Application");						  //加载Excel驱动
    excel.dynamicCall("SetVisible (bool Visible)", "false");	  //不显示窗体
    excel.setProperty("DisplayAlerts", false);					  //不显示任何警告信息。如果为true那么在关闭是会出现类似“文件已修改，是否保存”的提示

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
    m_saveExcel=true;
}

void ToolWidget::onSaveTxt(){
    QString logInfo1="Txt开始保存";
    m_pMainWin->getPWinVtkPresetWidget()->setWidget(logInfo1);
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
         if(object->GetUniqueType()==enDistance){
            CDistance* Distance=(CDistance*) object;
            out<<"类型：距离 名称："<<Distance->m_strAutoName<<Qt::endl;
            out<<"大小："<<abs(Distance->getdistance())<<"  上公差："<<Distance->getUptolerance()<<"  下公差："<<Distance->getUndertolerance()<<Qt::endl;
            QString qua;
            if(Distance->judge())
            {
                qua="合格";
            }else{
                qua="不合格";
            }
            out<<"是否合格："<<qua<<Qt::endl;
            out<<Qt::endl;
        }else if(object->GetUniqueType()==enAngle){
            CAngle* Angle=(CAngle*)object;
            out<<"类型：角度 名称："<<Angle->m_strAutoName<<Qt::endl;
            out<<"大小："<<abs(Angle->getAngle())<<Qt::endl;
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
    m_saveTxt=true;
}




void   ToolWidget::onSaveWord(){

    QString logInfo1="Word开始保存";
    m_pMainWin->getPWinVtkPresetWidget()->setWidget(logInfo1);

    QString path= getOutputPath("word");
    QString name=getTimeString();

    QString filePath=path+"/"+name+".docx";



    QStringList headers;
    headers << "类型" << "名称" << "数值" << "上公差" << "下公差" << "是否合格" ;
    auto& entitylist = m_pMainWin->m_EntityListMgr->getEntityList();
    int col = headers.size();
    int row = entitylist.size();
    QList<QList<QString>> dataAll;
    QList<QList<QString>> dataAccepted;
    QList<QList<QString>>  dataNotAccepted;
    int count_accpted=0,count_not_accepted=0;

    ExtractData(entitylist, dataAll, dataAccepted,dataNotAccepted);

    count_accpted=dataAccepted.size();
    count_not_accepted=dataNotAccepted.size();


    int HeadTitleSize=35;
    int TitleSize=24;
    int TextSize=20;



    QAxObject *word = new QAxObject("Word.Application");
    word->dynamicCall("SetVisible(bool)", false);
    QAxObject *documents = word->querySubObject("Documents");
    documents->dynamicCall("Add()");
    QAxObject *document = word->querySubObject("ActiveDocument");

    // 添加标题
    QAxObject *selection = word->querySubObject("Selection");
    selection->dynamicCall("TypeText(const QString&)", "工业测量报告");
    selection->dynamicCall("EndKey(QVariant)", 6);

    // 封面页
    {
        selection->dynamicCall("TypeText(const QString&)", "工业测量报告\n\n");
        selection->dynamicCall("EndKey(QVariant)", 6);

        // 插入图片
        QString imagePath = ":/style/ruler.png";
        QImage image(imagePath);
        if (!image.isNull()) {
            QAxObject* inlineShapes = document->querySubObject("InlineShapes");
            inlineShapes->dynamicCall("AddPicture(const QString&)", imagePath);
            selection->dynamicCall("EndKey(QVariant)", 6);
        } else {
            qDebug() << "无法加载图片：:/style/ruler.png";
        }

        // 插入副标题和描述性文本
        selection->dynamicCall("TypeText(const QString&)", "测量项目: 3D 测量系统\n");
        selection->dynamicCall("TypeText(const QString&)", "测量日期: " + QDateTime::currentDateTime().toString("yyyy-MM-dd") + "\n");
        selection->dynamicCall("TypeText(const QString&)", "测量单位: 三维工业测量软件开发组\n\n");
        selection->dynamicCall("TypeText(const QString&)", "本报告详细记录了使用3D测量系统进行的测量项目。\n");
        selection->dynamicCall("TypeText(const QString&)", "测量日期为" + QDateTime::currentDateTime().toString("yyyy-MM-dd") + "，\n");
        selection->dynamicCall("TypeText(const QString&)", "由三维工业测量软件开发组负责执行和分析。\n");
        selection->dynamicCall("EndKey(QVariant)", 6);
    }

    // 目录
    {
        selection->dynamicCall("TypeText(const QString&)", "目录\n\n");
        selection->dynamicCall("TypeText(const QString&)", "1. 测量数据概览\n");
        selection->dynamicCall("TypeText(const QString&)", "2. 数据分析\n");
        selection->dynamicCall("TypeText(const QString&)", "3. 测量结果图\n");
        selection->dynamicCall("TypeText(const QString&)", "4. 结论与建议\n");
        selection->dynamicCall("TypeText(const QString&)", "5. 附录\n\n");
        selection->dynamicCall("EndKey(QVariant)", 6);
    }

    // 测量数据概览
    {
        selection->dynamicCall("TypeText(const QString&)", "1. 测量数据概览\n\n");

        int row = dataAll.size() + 1;
        int col = headers.size();
        QAxObject *range = selection->querySubObject("Range()");
        QAxObject *table = document->querySubObject("Tables")->querySubObject("Add(QVariant, int, int, QVariant, QVariant)", range->asVariant(), row, col, 1, 1);

        // 插入表头
        for (int i = 0; i < headers.size(); ++i) {
            QAxObject *cell = table->querySubObject("Cell(int, int)", 1, i + 1);
            cell->querySubObject("Range")->dynamicCall("SetText(const QString&)", headers[i]);
        }

        // 插入数据
        for (int i = 0; i < dataAll.size(); ++i) {
            for (int j = 0; j < dataAll[i].size(); ++j) {
                QAxObject *cell = table->querySubObject("Cell(int, int)", i + 2, j + 1);
                cell->querySubObject("Range")->dynamicCall("SetText(const QString&)", dataAll[i][j]);
            }
        }
        //selection->dynamicCall("EndKey(QVariant)", 6);
        selection->dynamicCall("EndKey(QVariant)", 6);
    }

    // 数据分析
    {
        selection->dynamicCall("TypeText(const QString&)", "2. 数据分析\n\n");
        selection->dynamicCall("TypeText(const QString&)", "根据测量数据，我们检测出以下指标不合格:\n");

        int row = dataNotAccepted.size() + 1;
        int col = headers.size();
        QAxObject *range = selection->querySubObject("Range()");
        QAxObject *table = document->querySubObject("Tables")->querySubObject("Add(QVariant, int, int, QVariant, QVariant)", range->asVariant(), row, col, 1, 1);

        // 插入表头
        for (int i = 0; i < headers.size(); ++i) {
            QAxObject *cell = table->querySubObject("Cell(int, int)", 1, i + 1);
            cell->querySubObject("Range")->dynamicCall("SetText(const QString&)", headers[i]);
        }

        // 插入数据
        for (int i = 0; i < dataNotAccepted.size(); ++i) {
            for (int j = 0; j < dataNotAccepted[i].size(); ++j) {
                QAxObject *cell = table->querySubObject("Cell(int, int)", i + 2, j + 1);
                cell->querySubObject("Range")->dynamicCall("SetText(const QString&)", dataNotAccepted[i][j]);
            }
        }
        selection->dynamicCall("EndKey(QVariant)", 6);
    }

    // 测量结果图
    {
        selection->dynamicCall("TypeText(const QString&)", "3. 测量结果图\n\n");
        selection->dynamicCall("TypeText(const QString&)", "（1）全局对比\n");
        int i = 0;
        for (const QString &imagePath : imagePaths) {
            QImage image(imagePath);
            if (!image.isNull()) {
                QAxObject* inlineShapes = document->querySubObject("InlineShapes");
                inlineShapes->dynamicCall("AddPicture(const QString&)", imagePath);
                selection->dynamicCall("EndKey(QVariant)", 6);
                selection->dynamicCall("TypeText(const QString&)", "\n       图" + QString::number(++i) + " 全局对比图\n");

            } else {
                qDebug() << "无法加载图片：" << imagePath;
            }
        }
        selection->dynamicCall("TypeText(const QString&)", "（2）局部对比\n");
        for (const QString &imagePath : imagePaths_part) {
            QImage image(imagePath);
            if (!image.isNull()) {
                QAxObject* inlineShapes = document->querySubObject("InlineShapes");
                inlineShapes->dynamicCall("AddPicture(const QString&)", imagePath);
                selection->dynamicCall("EndKey(QVariant)", 6);
                selection->dynamicCall("TypeText(const QString&)", "\n       图" + QString::number(++i) + " 局部对比图\n");
            } else {
                qDebug() << "无法加载图片：" << imagePath;
            }
        }
        selection->dynamicCall("EndKey(QVariant)", 6);
    }

    // 结论与建议
    {
        selection->dynamicCall("TypeText(const QString&)", "4. 结论与建议\n\n");

        selection->dynamicCall("TypeText(const QString&)", "-共检测出"+ QString::number(count_not_accepted)+"个不合格的指标\n");
        selection->dynamicCall("TypeText(const QString&)", "-共检测出"+ QString::number(count_accpted)+"个合格的指标\n");

        selection->dynamicCall("EndKey(QVariant)", 6);
    }

    // 附录
    {
        selection->dynamicCall("TypeText(const QString&)", "5. 附录\n\n");
        selection->dynamicCall("TypeText(const QString&)", "参考文献:\n");
        selection->dynamicCall("TypeText(const QString&)", "- 3d工业测量软件开发组测量标准\n");
        selection->dynamicCall("TypeText(const QString&)", "- 测量系统用户手册\n");
        selection->dynamicCall("EndKey(QVariant)", 6);
    }

    // 保存文档

    if (!filePath.isEmpty()) {
        document->dynamicCall("SaveAs2(const QString&)", filePath);
        document->dynamicCall("Close()");
        word->dynamicCall("Quit()");
        delete selection;
        delete document;
        delete documents;
        delete word;
        QString logInfo="Word保存成功";
        m_pMainWin->getPWinVtkPresetWidget()->setWidget(logInfo);

    }

    lastCreatedWordFile=filePath;
    m_saveWord=true;

}

void ToolWidget::onSavePointCloud(){

    auto& entityList = m_pMainWin->m_EntityListMgr->getEntityList();

    QVector<CPointCloud*> compare_clouds;
    for(int i=0;i<entityList.size();i++){
        CEntity* entity=entityList[i];
        if(entity->getEntityType()==enPointCloud){
            CPointCloud* point_cloud=(CPointCloud*)entity;

            if(point_cloud->m_strAutoName.contains("对比")||
                point_cloud->m_strCName.contains("对比")){
                compare_clouds.push_back(point_cloud);
            }
        }
    }


    for(int i=0;i<compare_clouds.size();i++){
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud=compare_clouds[i]->m_pointCloud.makeShared();
  //+"("+QString::number(i+1)+")"
        //+"("+QString::number(i+1)+")"
        QString name=getTimeString();
        QString filePathPly=getOutputPath("ply")+"/"+name+".ply";

        QString filePathPcd=getOutputPath("pcd")+"/"+name+".pcd";

        //保存为pcl和pcd文件
        pcl::io::savePLYFile((filePathPly).toStdString(), *cloud);
        pcl::io::savePCDFileASCII((filePathPcd).toStdString(), *cloud);

    }
    QString logInfo="对比点云保存成功";
    m_pMainWin->getPWinVtkPresetWidget()->setWidget(logInfo);
}

static void WrongWidget(QString message,QString moreMessage="空");
void   ToolWidget::onSaveImage(){
    //得到路径名称
    QString filter = "PNG (*.png);;JPEG (*.jpg *.jpeg);;TIFF (*.tif *.tiff);;BMP (*.bmp)";
    QString fileName;
    if (!IsAuto) {
        QString path= getOutputPath("image");
        QString name=getTimeString();
        fileName=path+"/"+name+".png";

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

    LineConstructor constructor;
    CLine* newLine;
    bool createLine=false;
        {
        auto& entityList = m_pMainWin->m_EntityListMgr->getEntityList();
        newLine=(CLine*)constructor.create(entityList);
        if(newLine!=nullptr){
            addToList(newLine);
        }

    }
    if(!createLine&&newLine==nullptr){
        if(newLine==nullptr){
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

    PlaneConstructor constructor;
    CPlane* newPlane;
    bool createPlane=false;

   {
        auto& entityList = m_pMainWin->m_EntityListMgr->getEntityList();
        newPlane=(CPlane*)constructor.create(entityList);
        if(newPlane!=nullptr){
            addToList(newPlane);
        }
    }

    if(!createPlane&&newPlane==nullptr){
        {
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
        newPointCloud->parent.append(pointClouds[i]);
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

    if(!m_pMainWin->getPWinSetDataWidget()->getBeginFitting())  return;

    auto planeCloud=m_pMainWin->getPWinSetDataWidget()->getPlaneCloud();
    if(planeCloud==nullptr){
        qDebug()<<"拟合平面生成错误";
        return ;
    }
    auto plane=m_pMainWin->getPWinSetDataWidget()->getPlane();
    if(plane==nullptr){
        return;
    }

    PointConstructor p_constructor;
    CPoint *newPoint;
    newPoint=p_constructor.createPoint(point.x,point.y,point.z);
    newPoint->Form="识别";
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
    newPlane->rad=plane->getradius();
    newPlane->dis=plane->getdistance();
    newPlane->parent.push_back(newPoint);
    newPlane->isFind=true;
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

    if(!m_pMainWin->getPWinSetDataWidget()->getBeginFitting())  return;

    // 生成点云对象并添加到entitylist
    auto lineCloud=m_pMainWin->getPWinSetDataWidget()->getLineCloud();
    if(lineCloud==nullptr){
        qDebug()<<"拟合直线生成错误";
        return ;
    }
    auto line=m_pMainWin->getPWinSetDataWidget()->getLine();
    if(line==nullptr){
        return;
    }

    PointConstructor p_constructor;
    CPoint *newPoint;
    newPoint=p_constructor.createPoint(point.x,point.y,point.z);
    newPoint->Form="识别";
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
    newLine->parent.push_back(newPoint);
    newLine->dis=line->getDistance();
    if(newLine==nullptr){
        qDebug()<<"拟合直线生成错误";
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

    if(!m_pMainWin->getPWinSetDataWidget()->getBeginFitting())  return;

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

    if(!m_pMainWin->getPWinSetDataWidget()->getBeginFitting())  return;

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

    if(!m_pMainWin->getPWinSetDataWidget()->getBeginFitting())  return;

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

    if(!m_pMainWin->getPWinSetDataWidget()->getBeginFitting())  return;

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

    if(!m_pMainWin->getPWinSetDataWidget()->getBeginFitting())  return;

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
QVector<QString>& ToolWidget::getImagePaths_part(){
    return imagePaths_part;
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

        //qDebug() << "文件夹创建成功:" << path;
    } else {

        //qDebug() << "文件夹创建失败:" << path;
    }
}


QString ToolWidget::getOutputPath(QString kind){
   return "D:/output/" + kind;

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
QString ToolWidget::getlastCreatedImageFileFront(){
    return lastCreatedImageFileFront;
}
QString ToolWidget::getlastCreatedImageFileTop(){
    return lastCreatedImageFileTop;
}
QString ToolWidget::getlastCreatedImageFileRight(){
    return lastCreatedImageFileRight;
}

QAction *  ToolWidget::getAction_Checked(){
    if(Action_Checked)
        return(QAction*)Action_Checked;
    else
        return nullptr;
}
bool ToolWidget::IsFindPoint_Checked(){
    return Is_FindPoint_Cheked;
}


QStringList* ToolWidget::getSaveIconPath(){
    return &save_action_iconpath_list_;
}
QStringList* ToolWidget::getConstructIconPath(){
    return &construct_action_iconpath_list_;
}
QStringList* ToolWidget::getFindIconPath(){
    return &find_action_iconpath_list_;
}
QStringList* ToolWidget::getCoordIconPath(){
    return & coord_action_iconpath_list_;
}
QStringList* ToolWidget::getViewAngleIconPath(){
    return &view_angle_action_iconpath_list_;
}

