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
#include "filemanagerwidget.h"
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

//输出点云文件
#include <pcl/io/ply_io.h>  // 用于保存 .ply 文件
#include <pcl/io/pcd_io.h>  // 用于保存 .pcd 文件
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//从字符串提取数字
#include <QRegularExpression>
#include <QDebug>


int extractFirstNumber(const QString &str) {
    QRegularExpression re("(\\d+)"); // 匹配一个或多个数字
    QRegularExpressionMatch match = re.match(str);

    if (match.hasMatch()) {
        QString numStr = match.captured(1);
        return numStr.toInt(); // 返回第一个匹配的数字
    }

    return -1; // 如果没有找到数字，返回-1或其他默认值
}


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

    m_savePath=loadAddress();
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

    m_pMainWin->getPWinVtkPresetWidget()->setWidget("保存路径:"+m_savePath);

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
    // connect(save_actions_[save_action_name_list_.indexOf("pdf")],&QAction::triggered,this,&  ToolWidget::);
    // connect(save_actions_[save_action_name_list_.indexOf("image")],&QAction::triggered,this,&  ToolWidget::onSaveImage);
    connect(save_actions_[save_action_name_list_.indexOf("pointcloud")],&QAction::triggered,this,&  ToolWidget::onSavePointCloud);
    //打开
    connect(save_actions_[save_action_name_list_.indexOf("excel")], &QAction::triggered, this, &ToolWidget::onOpenExcel);
    connect(save_actions_[save_action_name_list_.indexOf("word")], &QAction::triggered, this, &ToolWidget::onSaveWord);
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
    if (!QDesktopServices::openUrl(QUrl::fromLocalFile(lastCreatedExcelFile_size))) {
        QString log="无法打开第二个Excel文件";
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


void  ToolWidget:: ExtractData(QVector<CEntity *>& entitylist,QList<QList<QString>>& dataAll){
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

void   ToolWidget::ExtractData(QVector<CEntity *>& entitylist,QList<QList<QString>>& dataAll,QList<QString>& data_PointCloud){

    for (int i = 0; i < entitylist.size(); i++) {
        QList<QString> inList;
        CEntity* entity=entitylist[i];
        qDebug()<<"提取"<<entity->m_strAutoName<<"数据";
        if(entity->GetUniqueType()==enDistance){
            CDistance* Distance=(CDistance*) entity;
            inList<<"距离";
            inList<<Distance->m_strAutoName;
            inList<<QString::number(abs(Distance->getdistance()),'f',6);
            inList<<QString::number(Distance->getUptolerance(),'f',6);
            inList<<QString::number(Distance->getUndertolerance(),'f',6);
            if (Distance->judge())
                inList<<"合格";
            else{
                inList<<"不合格";
            }
            qDebug()<<"提取照片"<<entity->m_strAutoName;
            SaveImage(entity);
            qDebug()<<"提取照片"<<entity->m_strAutoName<<"结束";

            inList<<QDir::toNativeSeparators(m_checkpoint_imagePath[entity]);

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
            else{
                inList<<"不合格";}
            qDebug()<<"提取照片"<<entity->m_strAutoName;
            SaveImage(entity);
            qDebug()<<"提取照片"<<entity->m_strAutoName<<"结束";

            inList<<QDir::toNativeSeparators(m_checkpoint_imagePath[entity]);
        }
        else if(entity->GetUniqueType()==enPointCloud){

            CPointCloud* point_cloud=(CPointCloud*)entity;
            if(point_cloud->isAlignCloud){//实测点云
            }else{
                continue;
            }
            //名字 是否可见 法向量 颜色
            data_PointCloud<<point_cloud->m_strAutoName<<"是"<<"无"<<"无";
            //盒维数
            qDebug()<<"读取外包盒";
            auto boxData=m_pMainWin->getPWinVtkWidget()->getBoundboxData(entity);
            qDebug()<<"外包盒读取成功";
            if(boxData.isEmpty()){
                boxData = QVector<double>(3,0);
                qDebug() << "外包盒信息读取错误";
            }

            //double boxData[3]={0,0,0};
            CPosition boxCenter= m_pMainWin->getPWinVtkWidget()->getBoundboxCenter(entity);
            qDebug()<<"外包盒中心读取成功";

            //CPosition boxCenter;
            data_PointCloud<<("X:"+QString::number(boxData[0])
                                +" Y:"+QString::number(boxData[1])
                                +" Z:"+QString::number(boxData[2]));
            //盒中心
            data_PointCloud<<("X:"+QString::number( boxCenter.x)
                                +" Y:"+QString::number( boxCenter.y)
                                +" Z:"+QString::number( boxCenter.z));
            //点云数量
            data_PointCloud<<QString::number(point_cloud->getPointCloudSize());
            //Global Shift
            data_PointCloud<<"("+QString::number(0.00)+";"+QString::number(0.00)+";"+QString::number(0.00)+")";
            //Global Scale
            data_PointCloud<<QString::number(1.00);
            //点云大小
            data_PointCloud<<QString::number(m_pMainWin->ActorPointSize);
            qDebug()<<"保存图片"<<entity->m_strAutoName;
            SaveImage(entity);
            qDebug()<<"保存图片"<<entity->m_strAutoName<<"成功";
            data_PointCloud<<QDir::toNativeSeparators(m_checkpoint_imagePath[entity]);
        }
        if(inList.size())
            dataAll.append(inList);
        qDebug()<<"提取"<<entity->m_strAutoName<<"数据结束";
    }
}
void  ToolWidget:: ExtractData(QVector<CEntity *>& entitylist,QList<QList<QString>>& dataAll,QList<QList<QString>>& data_accepted,QList<QList<QString>>& data_not_accepted){
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

void   ToolWidget::ExtractData(QVector<CEntity *>& entitylist,QList<Size_MeasurementData>&dataAll_size,Size_MeasurementData& data_pointCloud_size){

    int  pointCloudindex=0;
    for (int i = 0; i < entitylist.size(); i++) {
        Size_MeasurementData tmp;
        CEntity* entity=entitylist[i];
        if(entity->GetUniqueType()==enPointCloud){

            CPointCloud* point_cloud=(CPointCloud*)entity;
            if(point_cloud->isComparsionCloud){//对比点云
            }else{
                continue;
            }
            pointCloudindex++;
            // CEntity* parent=nullptr;
            CPointCloud* parent=nullptr;
            bool isGlobal=false;//是否为全局
            if(point_cloud->parent.size()!=0){//有源
                if(point_cloud->parent[0]->GetUniqueType()==enPointCloud)//来源是点云
                {
                    parent=(CPointCloud*)point_cloud->parent[0];
                    if(point_cloud->isCut){
                        isGlobal=false;
                    }
                }
            }

            auto boxData=m_pMainWin->getPWinVtkWidget()->getBoundboxData(entity);
            qDebug()<<"外包盒读取成功";
            if(boxData.isEmpty()){
                boxData = QVector<double>(3,0);
                qDebug() << "外包盒信息读取错误";
            }

            CPosition boxCenter= m_pMainWin->getPWinVtkWidget()->getBoundboxCenter(entity);
            // CPosition boxCenter;


            //名字 是否可见 法向量 颜色
            tmp.name=point_cloud->m_strAutoName;

            tmp.isVisible="True";
            tmp.isColorful="True";
            tmp.isNormal="True";

            QVector<double>distanceValue=m_pMainWin->getPWinVtkWidget()->getDistanceValue()[point_cloud];


            if(distanceValue.size()==3){
                tmp.max_error=QString::number(distanceValue[0]);
                tmp.min_error=QString::number(distanceValue[1]);
                tmp.average_error=QString::number(distanceValue[2]);
            }else{
                tmp.max_error=tmp.min_error=tmp.average_error=0;
            }


            tmp.three_dimensional=("长:"+QString::number(boxData[0])
                                     +" 宽:"+QString::number(boxData[1])
                                     +" 高:"+QString::number(boxData[2]));;
            tmp.center_point=("X:"+QString::number(boxCenter.x)
                                +" Y:"+QString::number(boxCenter.y)
                                +" Z:"+QString::number(boxCenter.z));


            SaveImage(entity,&tmp);
            tmp.imagePath=QDir::toNativeSeparators(QDir::toNativeSeparators(m_checkpoint_imagePath[entity]));//图片路径

            if(pointCloudindex==1){
                tmp.type="Global";
                tmp.pointNumber=QString::number(point_cloud->getPointCloudSize());
                data_pointCloud_size=tmp;


            }else{
                tmp.type="Local";
                tmp.num_triangularmesh="1";//局部独有
                dataAll_size.push_back(tmp);

            }


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


void ToolWidget::createDistanceMeasurementReport_Pdf()
{
    createFolder();
    // 记录检测编号
    static int count = 1;
    // 提取点云、监测点信息
    qDebug()<<"pdf";
    auto& entityList = m_pMainWin->m_EntityListMgr->getEntityList();
    // QVector<CEntity*> check_points;//存储监测点
    QVector<CPointCloud*> compare_clouds;//存储点云
    QList<QList<QString>> dataAll;//存储监测点测量

    QList<Size_MeasurementData> dataAll_size;//存储尺寸测量

    QList<QString>  data_pointCloud;//存储检测点中点云数据

    Size_MeasurementData  data_pointCloud_size;//存储尺寸测量的点云数据
    ExtractData(entityList, dataAll, data_pointCloud);
    ExtractData(entityList, dataAll_size, data_pointCloud_size);

    // 创建PDFWriter对象

    qDebug()<<"pdf1";
    QString path = getOutputPath("pdf");
    QString name = getTimeString();
    QString fileName = path + "/" + name + ".pdf";

    QPdfWriter pdfWriter(fileName);
    pdfWriter.setPageSize(QPageSize(QPageSize::A4));
    //pdfWriter.setFileName(fileName);

    // 创建QPainter对象
    QPainter painter;
    if (!painter.begin(&pdfWriter)) {
        qDebug() << "无法开始绘制PDF";
        return;
    }

    // 设置字体
    QFont font("Times New Roman", 10.5);
    painter.setFont(font);

    const int leftMargin = 50;
    const int rightMargin = 50;
    const int topMargin = 50;
    const int bottomMargin = 50;



    // 获取页面尺寸（单位：点）并转换为像素
    int resolution = pdfWriter.resolution(); // 分辨率：每英寸点数（DPI）
    QPageSize pageSize =QPageSize(QPageSize::A4); // 页面大小
    QSizeF pageSizeMM = pageSize.size(QPageSize::Millimeter); // 页面尺寸（毫米）

    // 将页面尺寸从毫米转换为像素
    double widthPixels = (pageSizeMM.width() / 25.4) * resolution;  // 1英寸 = 25.4毫米
    double heightPixels = (pageSizeMM.height() / 25.4) * resolution;

    // 计算内容区域（单位：像素）
    QRectF pageRect(leftMargin, topMargin,
                    widthPixels - leftMargin - rightMargin,
                    heightPixels - topMargin - bottomMargin);

    // 添加报告标题
    font.setPointSize(18);
    font.setBold(true);
    painter.setFont(font);
    painter.drawText(pageRect, Qt::AlignCenter, "Distance Measurement Report");
    painter.drawText(pageRect.adjusted(0, 20, 0, 0), Qt::AlignLeft, "检测物件: " + data_pointCloud[0]);
    painter.drawText(pageRect.adjusted(0, 40, 0, 0), Qt::AlignLeft, "检测日期: " + QDate::currentDate().toString("yyyy/M/d"));
    painter.drawText(pageRect.adjusted(0, 60, 0, 0), Qt::AlignLeft, "检测编号: " + QString("%1").arg(count++, 3, 10, QLatin1Char('0')));

    // 创建属性表格
    QStringList propertyHeaders = {"名称", "是否可见", "法向量", "颜色",
                                   "盒维数", "盒中心", "点云数量", "Golbal Shift",
                                   "Global Scale", "点云大小"};
    QStringList propertyValues = data_pointCloud;

    int tableY = 100;
    int rowHeight = 23;
    int colWidth = pageRect.width() / propertyHeaders.size();

    // 绘制表格标题行
    font.setBold(true);
    painter.setFont(font);
    qDebug()<<"pdf3";
    for (int col = 0; col < propertyHeaders.size(); ++col) {
        qDebug()<<col;
        QRectF cellRect(pageRect.left() + col * colWidth, tableY, colWidth, rowHeight);
        painter.drawRect(cellRect);
        painter.drawText(cellRect, Qt::AlignCenter, propertyHeaders[col]);
    }
    qDebug()<<"pdf4";
    // 绘制表格数据行
    font.setBold(false);
    painter.setFont(font);
    for (int row = 1; row < propertyValues.size(); ++row) {
        for (int col = 0; col < 2; ++col) {
            QRectF cellRect(pageRect.left() + col * colWidth, tableY + (row + 1) * rowHeight, colWidth, rowHeight);
            painter.drawRect(cellRect);
            if (col == 0) {
                painter.drawText(cellRect, Qt::AlignCenter, propertyHeaders[row-1]);
            } else {
                painter.drawText(cellRect, Qt::AlignCenter, propertyValues[row]);
            }
        }
    }
    qDebug()<<"pdf5";
    // 添加检测点数据
    QVector<MeasurementData> measurements;
    for (auto &dataList : dataAll) {
        MeasurementData data;
        data.type = dataList[0];
        data.pointNumber = QString::number(extractFirstNumber(dataList[1]));//检测点序号
        data.measuredValue = dataList[2];
        data.maxValue = dataList[3];
        data.minValue = dataList[4];
        data.isQualified = dataList[5];
        data.imagePath = QDir::toNativeSeparators(dataList[6]);
        measurements.push_back(data);
    }
    qDebug()<<"pdf6";
    for (const MeasurementData& data : measurements) {
        // 添加检测点标题
        font.setPointSize(14);
        font.setBold(true);
        painter.setFont(font);
        painter.drawText(pageRect.adjusted(0, tableY + (propertyValues.size() + 2) * rowHeight, 0, 0), Qt::AlignLeft, "检测点" + data.pointNumber);

        // 生成输出数据
        QStringList measureHeaders = {"检测点序号","类型", "测量值", "最大值", "最小值", "是否合格"};
        QStringList measureValues = {data.pointNumber,data.type, data.measuredValue, data.maxValue, data.minValue, data.isQualified};

        // 创建检测点表格
        int measureTableY = tableY + (propertyValues.size() + 3) * rowHeight;
        for (int col = 0; col < measureHeaders.size(); ++col) {
            QRectF cellRect(pageRect.left() + col * colWidth, measureTableY, colWidth, rowHeight);
            painter.drawRect(cellRect);
            painter.drawText(cellRect, Qt::AlignCenter, measureHeaders[col]);
        }

        // 绘制检测点表格数据行
        for (int row = 0; row < measureValues.size(); ++row) {
            for (int col = 0; col < 2; ++col) {
                QRectF cellRect(pageRect.left() + col * colWidth, measureTableY + (row + 1) * rowHeight, colWidth, rowHeight);
                painter.drawRect(cellRect);
                if (col == 0) {
                    painter.drawText(cellRect, Qt::AlignCenter, measureHeaders[row]);
                } else {
                    painter.drawText(cellRect, Qt::AlignCenter, measureValues[row]);
                }
            }
        }

        // 添加图片
        // 此处需要处理图片的加载和绘制，暂时省略

        tableY = measureTableY + (measureValues.size() + 2) * rowHeight;
    }

    // 添加整体检测部分
    font.setPointSize(14);
    font.setBold(true);
    painter.setFont(font);
    painter.drawText(pageRect.adjusted(0, tableY, 0, 0), Qt::AlignLeft, "整体检测");

    // 生成输出数据
    QStringList propertyHeaders_size = {"类型", "名称", "是否可视化", "是否有法向量",
                                        "是否有颜色", "外包盒三维", "外包盒中心点", "点的数目",
                                        "最大误差", "最小误差","平均误差"};
    QStringList propertyValues_size;
    propertyValues_size << data_pointCloud_size.type << data_pointCloud_size.name
                        << data_pointCloud_size.isVisible << data_pointCloud_size.isNormal << data_pointCloud_size.isColorful
                        << data_pointCloud_size.three_dimensional << data_pointCloud_size.center_point
                        << data_pointCloud_size.pointNumber << data_pointCloud_size.max_error << data_pointCloud_size.min_error
                        << data_pointCloud_size.average_error;

    qDebug()<<"pdf7";   // 创建全局检测属性表格
    int sizeTableY = tableY + 20;
    for (int col = 0; col < propertyHeaders_size.size(); ++col) {
        QRectF cellRect(pageRect.left() + col * colWidth, sizeTableY, colWidth, rowHeight);
        painter.drawRect(cellRect);
        painter.drawText(cellRect, Qt::AlignCenter, propertyHeaders_size[col]);
    }
    qDebug()<<"pdf8";
    // 绘制表格数据行
    for (int row = 0; row < propertyValues_size.size(); ++row) {
        for (int col = 0; col < 2; ++col) {
            QRectF cellRect(pageRect.left() + col * colWidth, sizeTableY + (row + 1) * rowHeight, colWidth, rowHeight);
            painter.drawRect(cellRect);
            if (col == 0) {
                painter.drawText(cellRect, Qt::AlignCenter, propertyHeaders_size[row]);
            } else {
                painter.drawText(cellRect, Qt::AlignCenter, propertyValues_size[row]);
            }
        }
    }

    // 绘制局部检测部分
    int num_size = 1;
    for (const Size_MeasurementData& data : dataAll_size) {
        // 添加局部检测标题
        font.setPointSize(14);
        font.setBold(true);
        painter.setFont(font);
        painter.drawText(pageRect.adjusted(0, sizeTableY + (propertyHeaders_size.size() + 2) * rowHeight, 0, 0), Qt::AlignLeft, "局部检测" + QString::number(num_size++));

        // 生成输出数据
        QStringList measureHeaders = {"类型","名称", "是否有颜色", "是否有法向量", "三角网格量", "外包盒三维","外包盒中点",
                                      "最大误差","最小误差","平均误差"};
        QStringList measureValues ;

        measureValues << data.type << data.name
                      << data.isColorful << data.isNormal << data.num_triangularmesh
                      << data.three_dimensional << data.center_point
                      << data.max_error << data.min_error
                      << data.average_error;

        // 创建局部检测表格
        int measureTableY = sizeTableY + (propertyHeaders_size.size() + 3) * rowHeight;
        for (int col = 0; col < measureHeaders.size(); ++col) {
            QRectF cellRect(pageRect.left() + col * colWidth, measureTableY, colWidth, rowHeight);
            painter.drawRect(cellRect);
            painter.drawText(cellRect, Qt::AlignCenter, measureHeaders[col]);
        }

        // 绘制局部检测表格数据行
        for (int row = 0; row < measureValues.size(); ++row) {
            for (int col = 0; col < 2; ++col) {
                QRectF cellRect(pageRect.left() + col * colWidth, measureTableY + (row + 1) * rowHeight, colWidth, rowHeight);
                painter.drawRect(cellRect);
                if (col == 0) {
                    painter.drawText(cellRect, Qt::AlignCenter, measureHeaders[row]);
                } else {
                    painter.drawText(cellRect, Qt::AlignCenter, measureValues[row]);
                }
            }
        }

        sizeTableY = measureTableY + (measureValues.size() + 2) * rowHeight;

        // 添加图片
        // 此处需要处理图片的加载和绘制，暂时省略

        // 分页处理
        if (&data != &dataAll_size.last()) {
            painter.end();
            pdfWriter.newPage();
            painter.begin(&pdfWriter);
            font.setPointSize(10.5);
            painter.setFont(font);
        }
    }

    // 结束绘制
    painter.end();

    QString logInfo="Pdf保存成功";
    m_pMainWin->getPWinVtkPresetWidget()->setWidget(logInfo);
    lastCreatedPdfFile=fileName;
    m_savePdf=true;
}


void   ToolWidget::onSavePdf(){
    /*

    createFolder();
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
*/
}





void   ToolWidget::onSaveExcel(){
    createFolder();
    QString logInfo1="Excel开始保存";
    m_pMainWin->getPWinVtkPresetWidget()->setWidget(logInfo1);

    QString path= getOutputPath("excell"+m_charBeforeDot);
    QString name="检测点信息_"+getTimeString();

    QString filePath=path+"/"+name+".xlsx";
    // QString filePath = QFileDialog::getSaveFileName(nullptr, QString("Save As"), "", QString("Excel(*.xlsx *.xls)"));
    // if (filePath.isEmpty()){
    //     return ;
    // }

    QStringList headers;
    headers << "检测点序号" <<"检测数值"<< "Max" << "Min"<<"是否合格";
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


    QAxObject *usedRange = workSheet->querySubObject("UsedRange");
    QAxObject *font = usedRange->querySubObject("Font");
    font->setProperty("Name", "宋体");  // 设置字体为宋体
    font->setProperty("Size", 11);     // 可选：设置字号（默认 11 号）

    // 只设置列标题（无其他格式）
    // 设置列标题（仅加粗，无其他格式）
    for (int i = 0; i < col; i++)
    {
        QAxObject *cell = workSheet->querySubObject("Cells(int, int)", 1, i + 1);
        cell->dynamicCall("SetValue(const QString&)", headers[i]);
        //cell->querySubObject("Font")->setProperty("Bold", true); // 仅保留加粗
    }

    // 处理数据（从第2行开始）
    int curRow = 2;
    foreach(QList<QString> inLst, dataAll) {
        for (int j = 0; j < headers.size(); j++) {
            QAxObject *cell = workSheet->querySubObject("Cells(int, int)", curRow, j + 1);
            cell->dynamicCall("SetValue(const QString&)", inLst[j+1]);
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
    createFolder();
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
    createFolder();
    createDistanceMeasurementReport();

}




void ToolWidget::createDistanceMeasurementReport()
{

    //记录检测编号
    static int count=1;
    //提取点云、监测点信息

    auto& entityList = m_pMainWin->m_EntityListMgr->getEntityList();
    if(entityList.size()==0)
    {
        QString logInfo="列表为空，无法保存word文档";
        m_pMainWin->getPWinVtkPresetWidget()->setWidget(logInfo);
    }
    //QVector<CEntity*> check_points;//存储监测点
    QVector<CPointCloud*> compare_clouds;//存储点云
    QList<QList<QString>> dataAll;//存储监测点测量

    QList<Size_MeasurementData>dataAll_size;//存储尺寸测量

    QList<QString>  data_pointCloud;//存储检测点中点云数据

    Size_MeasurementData  data_pointCloud_size;//存储尺寸测量的点云数据

    ExtractData(entityList, dataAll,data_pointCloud);

    ExtractData(entityList,dataAll_size,data_pointCloud_size);

    QAxObject* word = nullptr;
    QAxObject* document = nullptr;

    try {
        qDebug()<<"这里-10";
        // 1. 初始化Word应用程序
        word = new QAxObject("Word.Application");
        if (!word || word->isNull()) {
            throw std::runtime_error("无法启动Word应用程序");
        }
        word->setProperty("Visible", false);

        // 2. 创建新文档
        QAxObject* documents = word->querySubObject("Documents");
        if (!documents || documents->isNull()) {
            throw std::runtime_error("无法获取Documents对象");
        }

        document = documents->querySubObject("Add()");
        if (!document || document->isNull()) {
            throw std::runtime_error("无法创建新文档");
        }



        // 3. 获取Selection对象
        QAxObject* selection = word->querySubObject("Selection");
        if (!selection || selection->isNull()) {
            throw std::runtime_error("无法获取Selection对象");
        }
        // QAxObject* range = selection->querySubObject("Range");
        // range->querySubObject("ParagraphFormat")->setProperty("Alignment","wdAlignParagraphCenter");//水平居中
        // range->querySubObject("Cells")->setProperty("VerticalAlignment","wdCellAlignVerticalCenter");//垂直居中
        // 4. 设置页面边距
        QAxObject* pageSetup = document->querySubObject("PageSetup");
        pageSetup->setProperty("LeftMargin", 50);
        pageSetup->setProperty("RightMargin", 50);
        pageSetup->setProperty("TopMargin", 50);
        pageSetup->setProperty("BottomMargin", 50);


        // 获取页眉和页脚
        QAxObject* activeDocument = word->querySubObject("ActiveDocument");
        QAxObject* sections = activeDocument->querySubObject("Sections");
        QAxObject* firstSection = sections->querySubObject("First()");


        // 设置页眉
        QAxObject* header = firstSection->querySubObject("Headers(QVariant)", 1); // 1表示首页页眉
        QAxObject* headerRange = header->querySubObject("Range");

        // // 设置页眉文字
        headerRange->setProperty("Text", "距离尺寸测量报告");


        // 设置页眉文字格式
        QAxObject* headerFont = headerRange->querySubObject("Font");
        headerFont->setProperty("Name", "宋体");
        headerFont->setProperty("Size", 10.5);
        headerFont->setProperty("Bold", true);

        QAxObject* headerParagraphFormat = header->querySubObject("Range()")->querySubObject("ParagraphFormat");
        if (headerParagraphFormat) {
            QAxObject* borders = headerParagraphFormat->querySubObject("Borders");
            if (borders) {
                // 设置下边框样式为单实线，颜色为黑色，宽度为 1.5 磅
                borders->querySubObject("Item(int)", 3)->setProperty("LineStyle", 1); // 单实线
                borders->querySubObject("Item(int)", 3)->setProperty("LineWidth", 1.5);
                borders->querySubObject("Item(int)", 3)->setProperty("Color", 255); // 黑色
            }
        }
        qDebug()<<"这里-8";
        // 设置页脚
        QAxObject* footer = firstSection->querySubObject("Footers(QVariant)", 1); // 1表示首页页脚
        QAxObject* footerRange = footer->querySubObject("Range");

        // 插入页码
        footerRange->querySubObject("Fields")->dynamicCall("Add(QVariant,QVariant,QVariant,QVariant)",
                                                           footerRange->asVariant(), -1, "PAGE", true);

        // 设置页码格式
        QAxObject* footerFont = footerRange->querySubObject("Font");
        footerFont->setProperty("Name", "宋体");
        footerFont->setProperty("Size", 10);


        QAxObject* footerParagraph = footerRange->querySubObject("ParagraphFormat");
        footerParagraph->setProperty("Alignment", "wdAlignParagraphCenter");


        qDebug()<<"这里-7";

        // 5. 设置字体样式
        QAxObject* font = selection->querySubObject("Font");
        // font->setProperty("Name", "宋体");
        font->setProperty("Name", "Times New Roman");           // 英文字体
        font->setProperty("NameFarEast", "宋体");
            // 字号（五号）
        QAxObject* paragraphFormat = selection->querySubObject("ParagraphFormat");
        paragraphFormat->setProperty("Alignment", "wdAlignParagraphCenter");
        if (paragraphFormat) {
            paragraphFormat->setProperty("Alignment", "wdCellAlignVerticalCenter"); // 1 代表居中对齐
        } else {
            qDebug() << "无法获取 ParagraphFormat 对象";
        }

        // 6. 添加报告标题
        font->setProperty("Size", 18);
        font->setProperty("Bold", true);
        selection->dynamicCall("TypeText(const QString&)", "Distance Measurement Report");
        selection->dynamicCall("TypeParagraph()");

        paragraphFormat->setProperty("Alignment", 0);


        qDebug()<<"这里-5";

        // 7. 添加基本信息
        font->setProperty("Size", 10.5);
        font->setProperty("Bold", false);

        if(data_pointCloud.size()>0){
            selection->dynamicCall("TypeText(const QString&)", "检测物件: "+data_pointCloud[0]);
        }else{
            selection->dynamicCall("TypeText(const QString&)", "检测物件:未知");
        }
        selection->dynamicCall("TypeParagraph()");
        selection->dynamicCall("TypeText(const QString&)", "检测日期: " + QDate::currentDate().toString("yyyy/M/d"));
        selection->dynamicCall("TypeParagraph()");

        selection->dynamicCall("TypeText(const QString&)", "检测编号: "+QString("%1").arg(count++, 3, 10, QLatin1Char('0')));
        selection->dynamicCall("TypeParagraph()");

        // 8. 创建属性表格

        QStringList propertyHeaders = {"名称", "是否可见", "法向量", "颜色",
                                       "盒维数", "盒中心", "点云数量", "Golbal Shift",
                                       "Global Scale", "点云大小"};
        QStringList propertyValues =data_pointCloud;
        QAxObject* tables = document->querySubObject("Tables");
        if(propertyHeaders.size()<=propertyValues.size()){

            QAxObject* rangeForTable = selection->querySubObject("Range");
            QAxObject* propertyTable = tables->querySubObject("Add(QVariant, QVariant, QVariant, QVariant)",
                                                              rangeForTable->asVariant(), 11, 2);
            if (!propertyTable || propertyTable->isNull()) {
                throw std::runtime_error("无法创建属性表格");
            }


            // 设置表格样式
            propertyTable->setProperty("Style", "网格型");
            propertyTable->querySubObject("Rows")->setProperty("Height", 23);
            QAxObject* tableRange = propertyTable->querySubObject("Range");
            if(tableRange==nullptr){
                qDebug()<<"tableRange创建失败";
                return ;
            }


            tableRange->querySubObject("ParagraphFormat")->setProperty("Alignment","wdAlignParagraphCenter");//水平居中
            tableRange->querySubObject("Cells")->setProperty("VerticalAlignment","wdCellAlignVerticalCenter");//垂直居中





            for (int row = 2; row <= propertyHeaders.size()+1; ++row) {
                // 设置第一列（属性标题）的文本和居中
                QAxObject* cellHeader = propertyTable->querySubObject("Cell(int, int)", row, 1);
                QAxObject* rangeHeader = cellHeader->querySubObject("Range");
                rangeHeader->dynamicCall("SetText(const QString&)", propertyHeaders[row - 2]);

                // 设置第二列（属性值）的文本和居中
                QAxObject* cellValue = propertyTable->querySubObject("Cell(int, int)", row, 2);
                QAxObject* rangeValue = cellValue->querySubObject("Range");
                rangeValue->dynamicCall("SetText(const QString&)", propertyValues[row - 2]);


                // 合并单元格（除第5和第6行外）

            }

            int row=1;
            QAxObject* cell1 = propertyTable->querySubObject("Cell(int, int)", row, 1);
            QAxObject* cell2 = propertyTable->querySubObject("Cell(int, int)", row, 2);
            cell1->dynamicCall("Merge(QVariant)", cell2->asVariant());
            QAxObject* inlineShapes = document->querySubObject("InlineShapes");
            inlineShapes->dynamicCall("AddPicture(const QString&)",data_pointCloud[data_pointCloud.size()-1] );
            qDebug()<<"这2";


            //结束表格填写
            selection->dynamicCall("EndKey(QVariant)", 6);
            selection->dynamicCall("MoveDown()");
            selection->dynamicCall("InsertBreak(int)", 7);

        }


        // 9. 添加检测点数据
        QVector<MeasurementData> measurements;
        for(auto &dataList:dataAll){
            if(dataList.size()<7)continue;
            MeasurementData data;
            data.type=dataList[0];
            data.pointNumber=QString::number(extractFirstNumber(dataList[1]));//检测点序号
            data.measuredValue=dataList[2];
            data.maxValue=dataList[3];
            data.minValue=dataList[4];
            data.isQualified=dataList[5];
            data.imagePath= QDir::toNativeSeparators(dataList[6]);

            measurements.push_back(data);
        }

        for (const MeasurementData& data : measurements) {
            // 添加检测点标题
            font->setProperty("Size", 14);
            font->setProperty("Bold", true);
            selection->dynamicCall("TypeText(const QString&)", "检测点" + data.pointNumber);
            selection->dynamicCall("TypeParagraph()");
            font->setProperty("Size", 10.5);
            font->setProperty("Bold", false);
            //生成输出数据
            QStringList measureHeaders = {"检测点序号","类型", "测量值", "最大值", "最小值", "是否合格"};
            QStringList measureValues = {data.pointNumber,data.type, data.measuredValue, data.maxValue, data.minValue, data.isQualified};
            if(measureValues.size()==measureHeaders.size()){
                // 创建检测点表格
                QAxObject* measureRange = selection->querySubObject("Range");
                QAxObject* measureTable = tables->querySubObject("Add(QVariant, QVariant, QVariant, QVariant)",
                                                                 measureRange->asVariant(), measureHeaders.size()+1, 2);
                QAxObject* measureTableRange = measureTable->querySubObject("Range");
                if (!measureTable || measureTable->isNull()) {
                    throw std::runtime_error("无法创建检测点表格");
                }

                measureTable->setProperty("Style", "网格型");
                measureTable->querySubObject("Rows")->setProperty("Height", 23);


                measureTableRange->querySubObject("ParagraphFormat")->setProperty("Alignment","wdAlignParagraphCenter");//水平居中
                measureTableRange->querySubObject("Cells")->setProperty("VerticalAlignment","wdCellAlignVerticalCenter");//垂直居中

                for (int row = 2; row <= measureHeaders.size()+1; ++row) {
                    QAxObject* cell = measureTable->querySubObject("Cell(int, int)", row, 1);
                    QAxObject* range = cell->querySubObject("Range");

                    //设置内容
                    range->dynamicCall("SetText(const QString&)", measureHeaders[row-2]);
                    cell = measureTable->querySubObject("Cell(int, int)", row, 2);
                    range = cell->querySubObject("Range");
                    range->dynamicCall("SetText(const QString&)", measureValues[row-2]);

                }

                int row=1;
                QAxObject* cell1 = measureTable->querySubObject("Cell(int, int)", row, 1);
                QAxObject* cell2 = measureTable->querySubObject("Cell(int, int)", row, 2);
                cell1->dynamicCall("Merge(QVariant)", cell2->asVariant());
                QAxObject* inlineShapes = document->querySubObject("InlineShapes");
                inlineShapes->dynamicCall("AddPicture(const QString&)",data.imagePath==""?m_NULLimagePath:data.imagePath);
                selection->dynamicCall("EndKey(QVariant)", 6);


                //selection->dynamicCall("MoveDown()");
                selection->dynamicCall("InsertBreak(int)", 7); // 分页符
            }
        }

        QStringList propertyHeaders_size = {"类型", "名称", "是否可视化", "是否有法向量",
                                            "是否有颜色", "外包盒三维", "外包盒中心点", "点的数目",
                                            "最大误差", "最小误差","平均误差"};
        QStringList propertyValues_size;
        if(data_pointCloud_size.name!=""){


            propertyValues_size<<data_pointCloud_size.type<<data_pointCloud_size.name
                                <<data_pointCloud_size.isVisible<<data_pointCloud_size.isNormal<<data_pointCloud_size.isColorful
                                <<data_pointCloud_size.three_dimensional<<data_pointCloud_size.center_point
                                <<data_pointCloud_size.pointNumber<<data_pointCloud_size.max_error<<data_pointCloud_size.min_error
                                <<data_pointCloud_size.average_error;

            font->setProperty("Size", 14);
            font->setProperty("Bold", true);
            selection->dynamicCall("TypeText(const QString&)", "整体检测");
            selection->dynamicCall("TypeParagraph()");
            font->setProperty("Size", 10.5);
            font->setProperty("Bold", false);

            if(propertyHeaders_size.size()==propertyValues_size.size()){
                QAxObject* rangeForTable_size = selection->querySubObject("Range");
                QAxObject* propertyTable_size = tables->querySubObject("Add(QVariant, QVariant, QVariant, QVariant)",
                                                                       rangeForTable_size->asVariant(), propertyHeaders_size.size()+1, 2);
                if (!propertyTable_size || propertyTable_size->isNull()) {
                    throw std::runtime_error("无法创建属性表格");
                }

                // 设置表格样式
                propertyTable_size->setProperty("Style", "网格型");
                propertyTable_size->querySubObject("Rows")->setProperty("Height", 23);
                QAxObject* tableRange_size= propertyTable_size->querySubObject("Range");



                tableRange_size->querySubObject("ParagraphFormat")->setProperty("Alignment","wdAlignParagraphCenter");//水平居中
                tableRange_size->querySubObject("Cells")->setProperty("VerticalAlignment","wdCellAlignVerticalCenter");//垂直居中



                for (int row = 2; row <= propertyHeaders_size.size()+1; ++row) {
                    // 设置第一列（属性标题）的文本和居中
                    QAxObject* cellHeader = propertyTable_size->querySubObject("Cell(int, int)", row, 1);
                    QAxObject* rangeHeader = cellHeader->querySubObject("Range");
                    rangeHeader->dynamicCall("SetText(const QString&)", propertyHeaders_size[row - 2]);

                    // 设置第二列（属性值）的文本和居中
                    QAxObject* cellValue = propertyTable_size->querySubObject("Cell(int, int)", row, 2);
                    QAxObject* rangeValue = cellValue->querySubObject("Range");
                    rangeValue->dynamicCall("SetText(const QString&)", propertyValues_size[row - 2]);


                    // 合并单元格（除第5和第6行外）

                }

                QAxObject* cell1_size = propertyTable_size->querySubObject("Cell(int, int)", 1, 1);
                QAxObject* cell2_size = propertyTable_size->querySubObject("Cell(int, int)", 1, 2);
                cell1_size->dynamicCall("Merge(QVariant)", cell2_size->asVariant());
                QAxObject* inlineShapes_size = document->querySubObject("InlineShapes");
                inlineShapes_size->dynamicCall("AddPicture(const QString&)",data_pointCloud_size.imagePath==""?m_NULLimagePath:data_pointCloud_size.imagePath);




                //结束表格填写并换页
                selection->dynamicCall("EndKey(QVariant)", 6);
            }
        }


        int num_size=1;
        for (const Size_MeasurementData& data : dataAll_size) {
            selection->dynamicCall("InsertBreak(int)", 7); // 分页符
            // 添加检测点标题
            font->setProperty("Size", 14);
            font->setProperty("Bold", true);
            selection->dynamicCall("TypeText(const QString&)", "局部检测" +QString::number(num_size++));
            selection->dynamicCall("TypeParagraph()");
            font->setProperty("Size", 10.5);
            font->setProperty("Bold", false);
            //生成输出数据
            QStringList measureHeaders = {"类型","名称", "是否有颜色", "是否有法向量", "三角网格量", "外包盒三维","外包盒中点",
                                          "最大误差","最小误差","平均误差"};
            QStringList measureValues ;
            if(data.name!=""){


                measureValues<<data.type<<data.name
                              <<data.isColorful<<data.isNormal<<data.num_triangularmesh
                              <<data.three_dimensional<<data.center_point
                              <<data.max_error<<data.min_error
                              <<data.average_error;
                // 创建检测点表格
                QAxObject* measureRange = selection->querySubObject("Range");
                QAxObject* measureTable = tables->querySubObject("Add(QVariant, QVariant, QVariant, QVariant)",
                                                                 measureRange->asVariant(), measureHeaders.size()+1, 2);
                QAxObject* measureTableRange = measureTable->querySubObject("Range");
                if (!measureTable || measureTable->isNull()) {
                    throw std::runtime_error("无法创建检测点表格");
                }

                measureTable->setProperty("Style", "网格型");
                measureTable->querySubObject("Rows")->setProperty("Height", 23);


                measureTableRange->querySubObject("ParagraphFormat")->setProperty("Alignment","wdAlignParagraphCenter");//水平居中
                measureTableRange->querySubObject("Cells")->setProperty("VerticalAlignment","wdCellAlignVerticalCenter");//垂直居中

                for (int row = 2; row <= measureHeaders.size()+1; ++row) {
                    QAxObject* cell = measureTable->querySubObject("Cell(int, int)", row, 1);
                    QAxObject* range = cell->querySubObject("Range");

                    //设置内容
                    range->dynamicCall("SetText(const QString&)", measureHeaders[row-2]);
                    cell = measureTable->querySubObject("Cell(int, int)", row, 2);
                    range = cell->querySubObject("Range");
                    range->dynamicCall("SetText(const QString&)", measureValues[row-2]);

                }

                int row=1;
                QAxObject* cell1 = measureTable->querySubObject("Cell(int, int)", row, 1);
                QAxObject* cell2 = measureTable->querySubObject("Cell(int, int)", row, 2);
                cell1->dynamicCall("Merge(QVariant)", cell2->asVariant());
                QAxObject* inlineShapes = document->querySubObject("InlineShapes");
                inlineShapes->dynamicCall("AddPicture(const QString&)",data.imagePath==""?m_NULLimagePath:data.imagePath);
                selection->dynamicCall("EndKey(QVariant)", 6);


                // 最后一个检测点后不加分页
                if (&data != &dataAll_size.last()) {
                    selection->dynamicCall("EndKey(QVariant)", 6);

                    selection->dynamicCall("InsertBreak(int)", 7); // 分页符
                }
            }
        }

        // 10. 保存文档

        QString path= getOutputPath("叶片检测报告");
        QString name=getTimeString();

        QString filePath=path+"/"+name+".docx";
        QString defaultName = "分析报告_" + getTimeString() + ".docx";
        QString fileName =path+"/" +defaultName;

        if (!fileName.isEmpty()) {
            if (!fileName.endsWith(".docx", Qt::CaseInsensitive)) {
                fileName += ".docx";
            }
            document->dynamicCall("SaveAs(const QString&)", QDir::toNativeSeparators(fileName));
            //QMessageBox::information(nullptr, "完成", QString("报告已保存为:\n%1").arg(fileName));
        }

        // 11. 关闭文档
        document->dynamicCall("Close()");
        word->dynamicCall("Quit()");
        lastCreatedWordFile=fileName;
        m_saveWord=true;

        //QDesktopServices::openUrl(QUrl::fromLocalFile(fileName));//打开

    } catch (const std::exception& e) {
        QMessageBox::critical(nullptr, "错误", QString("生成报告时出错:\n%1").arg(e.what()));
        if (word && !word->isNull()) {
            if (document && !document->isNull()) {
                document->dynamicCall("Close(false)");
            }
            word->dynamicCall("Quit()");
        }
    }

    {
        QString path= getOutputPath("excell1");
        QString name="误差信息_"+getTimeString();

        QString filePath=path+"/"+name+".xlsx";
        // QString filePath = QFileDialog::getSaveFileName(nullptr, QString("Save As"), "", QString("Excel(*.xlsx *.xls)"));
        // if (filePath.isEmpty()){
        //     return ;
        // }

        QStringList headers;
        headers << "工件名称" << "最大误差" <<"最小误差"<< "平均误差" <<"是否合格";
        int col = headers.size();
        QList<QList<QString>> dataAll;
        auto& entitylist = m_pMainWin->m_EntityListMgr->getEntityList();
        //dataAll.append(headers);


        QList<QString>inList;
        if(data_pointCloud_size.name!=""){

            inList<<data_pointCloud_size.name<<data_pointCloud_size.max_error<<data_pointCloud_size.min_error<<data_pointCloud_size.average_error;

            if(data_pointCloud_size.max_error.toDouble()<0.001){
                inList<<"合格";
            }else{
                inList<<"不合格";
            }
            dataAll.append(inList);
        }

        for(const Size_MeasurementData& data : dataAll_size){
            if(data.name!=""){
                QList<QString>inList;

                inList<<data.name<<data.max_error<<data.min_error<<data.average_error;
                if(data.max_error.toDouble()<0.001){
                    inList<<"合格";
                }else{
                    inList<<"不合格";
                }
                dataAll.append(inList);
            }
        }
        QAxObject excel("Excel.Application");						  //加载Excel驱动
        excel.dynamicCall("SetVisible (bool Visible)", "false");	  //不显示窗体
        excel.setProperty("DisplayAlerts", false);					  //不显示任何警告信息。如果为true那么在关闭是会出现类似“文件已修改，是否保存”的提示

        QAxObject *workBooks = excel.querySubObject("WorkBooks");	  //获取工作簿集合
        workBooks->dynamicCall("Add");								  //新建一个工作簿
        QAxObject *workBook = excel.querySubObject("ActiveWorkBook"); //获取当前工作簿
        //QAxObject *workBook = excel.querySubObject("Open(QString&)", filePath); //获取当前工作簿
        QAxObject *workSheet = workBook->querySubObject("Sheets(int)", 1); //设置为 获取第一页 数据


        QAxObject *usedRange = workSheet->querySubObject("UsedRange");
        QAxObject *font = usedRange->querySubObject("Font");
        font->setProperty("Name", "宋体");  // 设置字体为宋体
        font->setProperty("Size", 11);     // 可选：设置字号（默认 11 号）

        // 只设置列标题（无其他格式）
        // 设置列标题（仅加粗，无其他格式）
        for (int i = 0; i < col; i++)
        {
            QAxObject *cell = workSheet->querySubObject("Cells(int, int)", 1, i + 1);
            cell->dynamicCall("SetValue(const QString&)", headers[i]);
            //cell->querySubObject("Font")->setProperty("Bold", true); // 仅保留加粗
        }

        // 处理数据（从第2行开始）
        int curRow = 2;
        foreach(QList<QString> inLst, dataAll) {
            for (int j = 0; j < headers.size(); j++) {
                QAxObject *cell = workSheet->querySubObject("Cells(int, int)", curRow, j + 1);
                cell->dynamicCall("SetValue(const QString&)", inLst[j]);
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
        lastCreatedExcelFile_size=filePath;
        //保存至filepath，注意一定要用QDir::toNativeSeparators将路径中的"/"转换为"\"，不然一定保存不了。
        workBook->dynamicCall("SaveAs(const QString&)", QDir::toNativeSeparators(filePath));
        workBook->dynamicCall("Close()");	//关闭工作簿
        excel.dynamicCall("Quit()");		//关闭excel
    }


    // 清理资源
    if (document) delete document;
    if (word) delete word;
}

void ToolWidget::onSavePointCloud(){
    /*
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
        QString filePathPly=getOutputPath("ply")+"/"+compare_clouds[i]->m_strAutoName+".ply";

        QString filePathPcd=getOutputPath("pcd")+"/"+compare_clouds[i]->m_strAutoName+".pcd";

        //保存为pcl和pcd文件
        pcl::io::savePLYFile((filePathPly).toStdString(), *cloud);
        pcl::io::savePCDFileASCII((filePathPcd).toStdString(), *cloud);

    }
    QString logInfo="对比点云保存成功";
    m_pMainWin->getPWinVtkPresetWidget()->setWidget(logInfo);
*/
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
    QString dateTimeString = QString("%1_%2_%3_%4_%5_%6")
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
    QDateTime currentDateTime = QDateTime::currentDateTime();

    // 获取年、月、日、小时、分钟和秒
    int year = currentDateTime.date().year();
    int month = currentDateTime.date().month();
    int day = currentDateTime.date().day();
    int hour = currentDateTime.time().hour();
    int minute = currentDateTime.time().minute();
    int second = currentDateTime.time().second();
    QString dateTimeString = QString("%1-%2-%3")
                                 .arg(year, 4, 10, QChar('0'))
                                 .arg(month, 2, 10, QChar('0'))
                                 .arg(day, 2, 10, QChar('0'));
    QString Path=m_savePath +"/"+dateTimeString;
    auto& entitylist=m_pMainWin->getEntityListMgr()->getEntityList();

    if(entitylist.size()!=0){
        for(auto & it :entitylist){
            if(it->m_strAutoName.contains("实测")){
                QString str=it->m_strAutoName;
                QString name="";
                int markerPos = str.indexOf(".");
                if (markerPos != -1 && markerPos < str.length() - 1) {
                    for(int i=0;i<markerPos;i++){
                        name+=str[i];
                    }
                    qDebug() << "实测点云名称为" << name;
                    return  Path+"/"+name+"/" +kind;
                }
            }
        }
    }
    return Path+"/"+ kind;

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

void ToolWidget::SaveImage(CEntity* entity,Size_MeasurementData* pointCloudData){
    qDebug() << "【步骤1】获取实体列表管理器";
    auto* entityListMgr = m_pMainWin->getEntityListMgr();
    if (entityListMgr == nullptr) {
        qDebug() << "错误：实体列表管理器（EntityListMgr）为空，终止操作";
        return;
    }
    qDebug() << "成功获取实体列表管理器，获取实体列表";
    auto& entitylist = entityListMgr->getEntityList();
    if(entitylist.size() == 0) {
        qDebug() << "警告：实体列表（entitylist）为空，终止操作";
        return;
    }
    qDebug() << "实体列表大小：" << entitylist.size();

    qDebug() << "【步骤2】获取对象列表管理器";
    auto* objectListMgr = m_pMainWin->getObjectListMgr();
    if (objectListMgr == nullptr) {
        qDebug() << "错误：对象列表管理器（ObjectListMgr）为空，终止操作";
        return;
    }
    qDebug() << "成功获取对象列表管理器，获取对象列表";
    QVector<CObject*>& objlist = objectListMgr->getObjectList();
    if(objlist.size() == 0) {
        qDebug() << "警告：对象列表（objlist）为空，终止操作";
        return;
    }
    qDebug() << "对象列表大小：" << objlist.size();

    qDebug() << "【步骤3】获取 actorToEntityMap";
    auto* actorToEntityMap = &m_pMainWin->getactorToEntityMap();
    if (actorToEntityMap == nullptr) {
        qDebug() << "错误：actorToEntityMap 为空，终止操作";
        return;
    }
    qDebug() << "成功获取 actorToEntityMap";
    auto& actorMap = *actorToEntityMap;

    qDebug() << "【步骤4】获取树状部件";
    auto* elementListWidget = m_pMainWin->getPWinElementListWidget();
    auto* treeWidget = elementListWidget ? elementListWidget->getTreeWidgetNames() : nullptr;
    if (treeWidget == nullptr) {
        qDebug() << "错误：树状部件（treeWidget）为空，终止操作";
        return;
    }
    qDebug() << "成功获取树状部件";

    std::vector<vtkActor*> actors;

    qDebug() << "【步骤5】获取渲染器组件";
    auto* vtkWidget = m_pMainWin->getPWinVtkWidget();
    if (vtkWidget == nullptr) {
        qDebug() << "错误：vtkWidget 为空，终止操作";
        return;
    }
    qDebug() << "成功获取 vtkWidget，获取渲染器";
    auto renderer = vtkWidget->getRenderer();
    if (renderer == nullptr) {
        qDebug() << "错误：渲染器（renderer）为空，终止操作";
        return;
    }
    qDebug() << "成功获取渲染器，获取视图道具集合";
    vtkPropCollection* propCollection = renderer->GetViewProps();
    if (propCollection == nullptr) {
        qDebug() << "错误：视图道具集合（propCollection）为空，终止操作";
        return;
    }
    qDebug() << "成功获取视图道具集合";

    QMap<vtkSmartPointer<vtkActor>, CEntity*>& actorToEntity = *actorToEntityMap;




    qDebug() << "【步骤7】处理树状部件选中项";
    QList<QTreeWidgetItem*> selectedItems = treeWidget->selectedItems();
    qDebug() << "选中项数量：" << selectedItems.size();
    for (QTreeWidgetItem* item : selectedItems) {
        if (item) {
            item->setSelected(false);
            qDebug() << "取消选中项：" << item->text(0);
        } else {
            qDebug() << "警告：选中项为空，跳过";
        }
    }

    qDebug() << "【步骤8】对准元素（entity）";
    if (entity) { // 假设 entity 是外部传入或类成员
        vtkWidget->FocusOnActor(entity);
        qDebug() << "成功对准元素：" << entity->m_strAutoName;
    } else {
        qDebug() << "错误：元素（entity）为空，无法对准";
    }

    qDebug() << "【步骤9】显示文本框";
    for(int i = 0; i < objlist.size(); i++){
        auto* obj = objlist[i];
        if (!obj || !entity) {
            continue;
        }
        bool match = (obj->m_strAutoName == entity->m_strAutoName ||
                      obj->m_strCName == entity->m_strAutoName ||
                      entity->m_strAutoName.contains(obj->m_strAutoName) ||
                      entity->m_strAutoName.contains(obj->m_strCName));
        if (match) {
            QTreeWidgetItem *item = treeWidget->topLevelItem(i);
            if (item) {
                item->setSelected(true);
                treeWidget->setCurrentItem(item);
                break;
            }
        }
    }

    if(pointCloudData == nullptr && entity && !entity->m_strAutoName.contains("实测")){
        if (elementListWidget) {
            elementListWidget->showInfotext();
        } else {

        }
    }

    qDebug() << "【步骤11】隐藏/显示元素";
    for(int i = 0; i < entitylist.size(); i++){
        auto currentEntity = entitylist[i];
        if (!currentEntity || !entity) {
            continue;
        }
        if(entity==currentEntity){
            continue;
        }

        bool Head = true;
        //处理父元素

        // for(auto* parent : entity->parent){ // 需确保 parent 容器不为空
        //     if (!parent) {
        //         continue;
        //     }
        //     bool parentMatch = (currentEntity->m_strAutoName == parent->m_strAutoName ||
        //                         currentEntity->m_strCName == parent->m_strAutoName ||
        //                         parent->m_strAutoName.contains(currentEntity->m_strAutoName) ||
        //                         parent->m_strAutoName.contains(currentEntity->m_strCName));
        //     if (parentMatch) {
        //         Head = false;
        //         break;
        //     }
        // }
        //处理点云

        if(currentEntity->GetUniqueType() == enPointCloud){
            auto pointCloud = dynamic_cast<CPointCloud*>(currentEntity);
            //保存图像的entity不是点云
            if(entity->GetUniqueType() == enDistance||entity->GetUniqueType() == enAngle)//距离或者角度
            {
                if ( pointCloud->isModelCloud ) {//当前点云是标准点云
                    Head = false;
                }
            }

        }


        if(Head){
            auto actor = actorToEntity.key(currentEntity, nullptr);
            if(actor){
                currentEntity->SetSelected(false);
                actor->SetVisibility(0); 
            }
        }
        else{
            if (vtkWidget) {
                vtkWidget->onHighLightActor(currentEntity);//高亮元素
            }

            auto actor = actorToEntity.key(currentEntity, nullptr);
            if(currentEntity) {
                currentEntity->SetSelected(false);
            }
            if(actor){
                actor->SetVisibility(true);
            }
        }
    }

    qDebug() << "【步骤12】高亮当前元素";
    if (entity) {

        if (vtkWidget) {
            vtkWidget->onHighLightActor(entity);
        }
        entity->SetSelected(true);

        auto actor = actorToEntity.key(entity, nullptr);
        if(actor){
            actor->SetVisibility(true);
        }
    }

    qDebug() << "【步骤13】生成图片路径";
    QString path = getOutputPath("image");
    if(entity->m_strAutoName.contains("对比"))
    {
        path = getOutputPath("image");
    }


    qDebug() << "输出路径：" << path;
    QString name = entity ? entity->m_strAutoName + getTimeString() : "default_entity" + getTimeString();
    QString fileName = path + "/" + name + ".png";
    qDebug() << "生成文件名：" << fileName;




    if(pointCloudData){
        if (vtkWidget) {
            vtkWidget->ShowColorBar(pointCloudData->min_error.toDouble(), pointCloudData->max_error.toDouble());
        }
    }

    qDebug() << "【步骤16】保存图片";

    if(entity->m_strAutoName.contains("对比"))
    {

        QString fileName = path + "/" + name;
        vtkSmartPointer<vtkRenderWindow> renderWindow=m_pMainWin->getPWinVtkWidget()->getRenderWindow();
        m_pMainWin->onRightViewClicked();
        //renderWindow->Render();
        SaveImage(fileName+"_Right"+".png","png");
        m_pMainWin->onTopViewClicked();
        //renderWindow->Render();
        SaveImage(fileName+"_Top"+".png","png");
        m_pMainWin->onFrontViewClicked();
        //renderWindow->Render();
        SaveImage(fileName+"_Front"+".png","png");
        fileName=fileName+"_Front"+".png";
        m_checkpoint_imagePath[entity] = fileName;

        lastCreatedImageFileFront=fileName+"_Front"+".png";
        lastCreatedImageFileTop=fileName+"_Top"+".png";
        lastCreatedImageFileRight=fileName+"_Right"+".png";

    }else{
        SaveImage(fileName, "png");
        m_checkpoint_imagePath[entity] = fileName;
    }

    qDebug() << "图片保存完成：" << fileName;

    // 恢复被隐藏的元素（原注释代码）
    // qDebug() << "【步骤17】恢复所有元素";
    // if (auto* fileManagerWidget = m_pMainWin->getPWinFileManagerWidget()) {
    //     fileManagerWidget->allRecover();
    //     qDebug() << "元素恢复成功";
    // } else {
    //     qDebug() << "警告：文件管理器部件为空，无法恢复元素";
    // }

    qDebug() << "【步骤18】关闭文本框";
    if (elementListWidget) {
        elementListWidget->closeInfotext();
        qDebug() << "关闭信息文本框";
    } else {
        qDebug() << "警告：元素列表部件为空，无法关闭文本框";
    }

    qDebug() << "【步骤19】更新渲染信息";
    if (vtkWidget) {
        vtkWidget->UpdateInfo();
        qDebug() << "渲染信息更新完成";
    } else {
        qDebug() << "警告：vtkWidget 为空，无法更新渲染信息";
    }
}

void ToolWidget:: createFolder(){
    auto& entitylist=m_pMainWin->getEntityListMgr()->getEntityList();


    QString charBeforeDot="A";
    for(auto entity: entitylist ){
        if(entity->GetUniqueType()==enPointCloud){
            CPointCloud* pointCloud=( CPointCloud*)entity;
            if(pointCloud->isMeasureCloud||pointCloud->m_strAutoName.contains("实测")){
                QString str=pointCloud->m_strAutoName;
                charBeforeDot="A";
                int markerPos = str.indexOf("#");
                if (markerPos != -1 && markerPos < str.length() - 1) {
                    QChar firstChar = str.at(markerPos + 1);
                    qDebug() << "# 后的第一个字符是:" << firstChar;
                    charBeforeDot=firstChar;

                } else {
                    qDebug() << "字符串中不存在# 或者# 后没有字符";
                }
            }
        }
    }
    m_charBeforeDot=charBeforeDot;
    createFolder(m_savePath);
    createFolder(getOutputPath("excell1"));
    createFolder(getOutputPath("excell"+charBeforeDot));
    createFolder(getOutputPath("image"));

    createFolder(getOutputPath("叶片检测报告"));

}
void ToolWidget::createFolderWithDialog()
{

    auto& entitylist=m_pMainWin->getEntityListMgr()->getEntityList();
    QWidget* parent = nullptr;
    const QString& defaultDir = QDir::homePath();
    const QString& defaultFolderName = "NewFolder";

    // 弹出文件选择对话框，让用户选择保存位置
    QString selectedDir = QFileDialog::getExistingDirectory(
        parent,
        QObject::tr("选择文件夹保存位置"),
        defaultDir,
        QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks
        );


    if (selectedDir.isEmpty()) {
        // 用户取消了对话框
        return ;
    }

    bool ok;
    QString folderName = QInputDialog::getText(
        parent,
        QObject::tr("输入文件夹名称"),
        QObject::tr("请输入文件夹名称:"),
        QLineEdit::Normal,
        defaultFolderName,
        &ok
        );

    if (!ok || folderName.isEmpty()) {
        // 用户取消了输入或输入为空
        return ;
    }

    // 构建完整的文件夹路径
    QString newFolderPath = QDir(selectedDir).filePath(folderName);

    // 检查文件夹是否已存在
    if (QDir(newFolderPath).exists()) {
        //m_pMainWin->getPWinVtkPresetWidget()->setWidget("新建文件夹并入已有文件夹");
    }

    // 创建文件夹
    m_savePath=newFolderPath;
    saveAddress(m_savePath);
    createFolder();

    m_pMainWin->getPWinVtkPresetWidget()->setWidget("保存路径:"+getOutputPath(""));

}


void ToolWidget::saveAddress(const QString &address) {
    QSettings settings("3D", "3D_path");
    settings.setValue("lastAddress", address);
}

QString ToolWidget::loadAddress() {
    QSettings settings("3D", "3D_path");
    return settings.value("lastAddress", "z:/result").toString(); // 默认值空字符串
}
