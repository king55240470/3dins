#include"toolwidget.h"
#include"toolaction.h"
#include <QtWidgets/QMainWindow>
#include <QMenu>
#include<QString>

#include<QPainter>
#include<QTableWidget>
#include <QMessageBox>
#include<QMenuBar>
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

#include <pcl/point_cloud.h>     // PCL 的点云类
#include <pcl/visualization/pcl_visualizer.h> // PCL 的可视化工具

#include "pointfitting/fittingplane.h"//拟合平面算法
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

int getImagePaths(const QString& directory, QStringList &iconPaths, QStringList &iconNames);

ToolWidget::ToolWidget(QWidget *parent)
    : QWidget(parent) {


    // m_save.m_pathList<<":/component/save/excel.png"<< ":/component/save/pdf.jpg"<< ":/component/save/txt.jpg"<< ":/component/save/word.jpg"<<":/component/save/image.jpg";
    // m_construct.m_pathList<<":/component/construct/point.jpg"<<":/component/construct/line.jpg"<<":/component/construct/circle.jpg"<<   ":/component/construct/plan.jpg"<<  ":/component/construct/rectangle.jpg"<<":/component/construct/cylinder.jpg"<< ":/component/construct/cone.jpg"<< ":/component/construct/sphere.jpg"<<":/component/construct/distance.png";
    // m_find.m_pathList<<":/component/construct/point.jpg"<<":/component/construct/line.jpg"<<":/component/construct/circle.jpg"<<   ":/component/construct/plan.jpg"<<  ":/component/construct/rectangle.jpg"<<":/component/construct/cylinder.jpg"<< ":/component/construct/cone.jpg"<< ":/component/construct/sphere.jpg";
    // m_coord.m_pathList<<":/component/coord/create.png"<<  ":/component/coord/spin.jpg"<<":/component/coord/save.png";
    // m_viewAngle.m_pathList<<":/component/viewangle/front.png"<<":/component/viewangle/up.png"<<":/component/viewangle/right.png"<<":/component/viewangle/isometric.png";


    // m_save.m_nameList<<"excel"<< "pdf"<< "txt"<< "word"<<"image";
    // m_construct.m_nameList<<"点"<<"线"<<"圆"<<"平面"<<"矩形"<<"圆柱"<<"圆锥"<<"球形"<<"距离";
    // m_find.m_nameList<<"点"<<"线"<<"圆"<<"平面"<<"矩形"<<"圆柱"<<"圆锥"<<"球形";;
    // m_coord.m_nameList<<"创建坐标系"<<"旋转坐标系"<<"保存坐标系";
    // m_viewAngle.m_nameList<<"主视角"<<"俯视角"<<"侧视角"<<"立体视角";
    // // 在Unique 的类中实现QAction的创建和初始化
    // //m_save.loadAction();

    // m_save.setName("保存");
    // m_construct.setName("构造");
    // m_find.setName("识别");
    // m_coord.setName("坐标系");
    // m_viewAngle.setName("视角");
    // //将每个工具栏加入汇总
    // m_toolBarGather.addUniqueToolBar(&m_find);
    // m_toolBarGather.addUniqueToolBar(&m_construct);
    // m_toolBarGather.addUniqueToolBar(&m_coord);
    // m_toolBarGather.addUniqueToolBar(&m_save);
    // m_toolBarGather.addUniqueToolBar(&m_viewAngle);
    // //向列表中添加工具栏的所有内容
    // //每行工具栏的图标数目
    // m_toolBarGather.setSingalToolBarActionNum(9);

    // m_construct.loadAction();
    // m_find.loadAction();
    // m_coord.loadAction();
    // m_viewAngle.loadAction();
    // m_save.loadAction();

    // m_toolBarGather.addToWidget(this);


    QVBoxLayout *layout = new QVBoxLayout(this);

    m_pMainWin =(MainWindow*)parent;



    resize(400,250);

    m_nSaveActionNum=5;
    m_nConstructActionNum=9;
    m_nFindActionNum=8;
    m_nCoordActionNum=3;
    m_nViewAngleActionNum=4;

    //静态保存图片路径和名称



    save_action_iconpath_list_<<":/component/save/excel.png"<< ":/component/save/pdf.jpg"<< ":/component/save/txt.jpg"<< ":/component/save/word.jpg"<<":/component/save/image.jpg";
    construct_action_iconpath_list_<<":/component/construct/point.jpg"<<":/component/construct/line.jpg"<<":/component/construct/circle.jpg"<<   ":/component/construct/plan.jpg"<<  ":/component/construct/rectangle.jpg"<<":/component/construct/cylinder.jpg"<< ":/component/construct/cone.jpg"<< ":/component/construct/sphere.jpg"<<":/component/construct/distance.png";
    find_action_iconpath_list_<<":/component/construct/point.jpg"<<":/component/construct/line.jpg"<<":/component/construct/circle.jpg"<<   ":/component/construct/plan.jpg"<<  ":/component/construct/rectangle.jpg"<<":/component/construct/cylinder.jpg"<< ":/component/construct/cone.jpg"<< ":/component/construct/sphere.jpg";
    coord_action_iconpath_list_<<":/component/coord/create.png"<<  ":/component/coord/spin.jpg"<<":/component/coord/save.png";
    view_angle_action_iconpath_list_<<":/component/viewangle/front.png"<<":/component/viewangle/up.png"<<":/component/viewangle/right.png"<<":/component/viewangle/isometric.png";





    save_action_name_list_<<"excel"<< "pdf"<< "txt"<< "word"<<"image";
    construct_action_name_list_<<"点"<<"线"<<"圆"<<"平面"<<"矩形"<<"圆柱"<<"圆锥"<<"球形"<<"距离";
    find_action_name_list_<<"点"<<"线"<<"圆"<<"平面"<<"矩形"<<"圆柱"<<"圆锥"<<"球形";;
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

    m_nToolbarNum =(allActionNum/SingalToolBarActionNum)+5;

    toolBars=new QToolBar*[m_nToolbarNum];


    for(int i=0;i<m_nToolbarNum;i++){

        toolBars[i]=new QToolBar(this);
        toolBars[i]->setIconSize(QSize(28,28));
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

void ToolWidget::clearToolWidget(){

    //清空
    for(int i=0;i<m_nToolbarNum;i++){
        clearToolBar(toolBars[i]);
    }

}
void ToolWidget::createToolWidget(){
    int toolbar_index=-1;
    int lastToolBar_index=0;


    layout()->addWidget(new QLabel("识别:"));
    toolbar_index= addFindActions(find_action_name_list_,m_nFindActionNum, toolbar_index);
    for(int i=lastToolBar_index;i<=toolbar_index;i++){
        layout()->addWidget(toolBars[i]);
    }
    lastToolBar_index=toolbar_index+1;


    layout()->addWidget(new QLabel("构造:"));
    toolbar_index= addConstructActions(construct_action_name_list_,m_nConstructActionNum, toolbar_index);
    for(int i=lastToolBar_index;i<=toolbar_index;i++){
        layout()->addWidget(toolBars[i]);
    }
    lastToolBar_index=toolbar_index+1;


    layout()->addWidget(new QLabel("保存:"));
    toolbar_index= addSaveActions(save_action_name_list_,m_nSaveActionNum, toolbar_index);
    for(int i=lastToolBar_index;i<=toolbar_index;i++){
        layout()->addWidget(toolBars[i]);
    }
    lastToolBar_index=toolbar_index+1;


    layout()->addWidget(new QLabel("坐标系:"));
    toolbar_index= addCoordActions(coord_action_name_list_,m_nCoordActionNum, toolbar_index);
    for(int i=lastToolBar_index;i<m_nToolbarNum;i++){
        layout()->addWidget(toolBars[i]);
    }
    lastToolBar_index=toolbar_index+1;

    layout()->addWidget(new QLabel("视角:"));
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

    // connect(m_find.m_actionList[m_find.m_nameList.indexOf("点")],&QAction::triggered,this,&   ToolWidget::onFindPoint);
    // connect(m_find.m_actionList[m_find.m_nameList.indexOf("线")],&QAction::triggered,this,&   ToolWidget::onFindLine);
    // connect(m_find.m_actionList[m_find.m_nameList.indexOf("圆")],&QAction::triggered,this,&   ToolWidget::onFindCircle);
    // connect(m_find.m_actionList[m_find.m_nameList.indexOf("平面")],&QAction::triggered,this,&  ToolWidget:: onFindPlane);
    // connect(m_find.m_actionList[m_find.m_nameList.indexOf("矩形")],&QAction::triggered,this,&   ToolWidget::onFindRectangle);
    // connect(m_find.m_actionList[m_find.m_nameList.indexOf("圆柱")],&QAction::triggered,this,&   ToolWidget::onFindCylinder);
    // connect(m_find.m_actionList[m_find.m_nameList.indexOf("圆锥")],&QAction::triggered,this,&   ToolWidget::onFindCone);
    // connect(m_find.m_actionList[m_find.m_nameList.indexOf("球形")],&QAction::triggered,this,&   ToolWidget::onFindSphere);

    // //构造
    // connect(m_construct.m_actionList[m_construct.m_nameList.indexOf("点")],&QAction::triggered,this,& ToolWidget::onConstructPoint);
    // connect(m_construct.m_actionList[m_construct.m_nameList.indexOf("线")],&QAction::triggered,this,&ToolWidget::onConstructLine);
    // connect(m_construct.m_actionList[m_construct.m_nameList.indexOf("圆")],&QAction::triggered,this,&ToolWidget::onConstructCircle);
    // connect(m_construct.m_actionList[m_construct.m_nameList.indexOf("平面")],&QAction::triggered,this,& ToolWidget::onConstructPlane);
    // connect(m_construct.m_actionList[m_construct.m_nameList.indexOf("矩形")],&QAction::triggered,this,& ToolWidget::onConstructRectangle);
    // connect(m_construct.m_actionList[m_construct.m_nameList.indexOf("圆柱")],&QAction::triggered,this,&  ToolWidget::onConstructCylinder);
    // connect(m_construct.m_actionList[m_construct.m_nameList.indexOf("圆锥")],&QAction::triggered,this,&  ToolWidget::onConstructCone);
    // connect(m_construct.m_actionList[m_construct.m_nameList.indexOf("球形")],&QAction::triggered,this,&  ToolWidget::onConstructSphere);
    // connect(m_construct.m_actionList[m_construct.m_nameList.indexOf("距离")],&QAction::triggered,this,&  ToolWidget::onConstructDistance);

    // //保存
    // connect(m_save.m_actionList[m_save.m_nameList.indexOf("excel")],&QAction::triggered,this,&  ToolWidget::onSaveExcel);
    // connect(m_save.m_actionList[m_save.m_nameList.indexOf("word")],&QAction::triggered,this,&  ToolWidget::onSaveWord);
    // connect(m_save.m_actionList[m_save.m_nameList.indexOf("txt")],&QAction::triggered,this,&  ToolWidget::onSaveTxt);
    // connect(m_save.m_actionList[m_save.m_nameList.indexOf("pdf")],&QAction::triggered,this,&  ToolWidget::onSavePdf);
    // connect(m_save.m_actionList[m_save.m_nameList.indexOf("image")],&QAction::triggered,this,&  ToolWidget::onSaveImage);

    // // connect(save_actions_[save_action_name_list_.indexOf("txt")],&QAction::triggered,this,&  ToolWidget::onSaveTxt);


    // //坐标系
    // connect(m_coord.m_actionList[m_coord.m_nameList.indexOf("创建坐标系")],&QAction::triggered,this,[&](){
    //     m_pMainWin->on2dCoordOriginAuto(); //创建临时坐标系
    // });
    // connect(m_coord.m_actionList[m_coord.m_nameList.indexOf("旋转坐标系")],&QAction::triggered,this,[&](){
    //     //tool_widget::onSpinCoord();
    //     m_pMainWin->on2dCoordSetRightX(); // x轴摆正
    // });
    // connect(m_coord.m_actionList[m_coord.m_nameList.indexOf("保存坐标系")],&QAction::triggered,this,[&](bool){
    //     //tool_widget::onSaveCoord();
    //     m_pMainWin->on2dCoordSave();
    // });
    // //视角
    // connect(m_viewAngle.m_actionList[m_viewAngle.m_nameList.indexOf("主视角")],&QAction::triggered,this,[&](){
    //     //tool_widget::onFrontViewAngle();
    //     m_pMainWin->onFrontViewClicked();
    // });
    // connect(m_viewAngle.m_actionList[m_viewAngle.m_nameList.indexOf("俯视角")],&QAction::triggered,this, [&](){
    //     //tool_widget::onUpViewAngle();
    //     m_pMainWin->onTopViewClicked();
    // });
    // connect(m_viewAngle.m_actionList[m_viewAngle.m_nameList.indexOf("侧视角")],&QAction::triggered,[&](){
    //     //tool_widget::onRightViewAngle();
    //     m_pMainWin->onRightViewClicked();
    // });
    // connect(m_viewAngle.m_actionList[m_viewAngle.m_nameList.indexOf("立体视角")],&QAction::triggered,[&](){
    //     //tool_widget::onIsometricViewAngle();
    //     m_pMainWin->onIsometricViewClicked();
    // });


    //识别
    connect(find_actions_[find_action_name_list_.indexOf("点")],&QAction::triggered,this,&   ToolWidget::onFindPoint);
    connect(find_actions_[find_action_name_list_.indexOf("线")],&QAction::triggered,this,&   ToolWidget::onFindLine);
    connect(find_actions_[find_action_name_list_.indexOf("圆")],&QAction::triggered,this,&   ToolWidget::onFindCircle);
    connect(find_actions_[find_action_name_list_.indexOf("平面")],&QAction::triggered,this,&  ToolWidget:: onFindPlane);
    connect(find_actions_[find_action_name_list_.indexOf("矩形")],&QAction::triggered,this,&   ToolWidget::onFindRectangle);
    connect(find_actions_[find_action_name_list_.indexOf("圆柱")],&QAction::triggered,this,&   ToolWidget::onFindCylinder);
    connect(find_actions_[find_action_name_list_.indexOf("圆锥")],&QAction::triggered,this,&   ToolWidget::onFindCone);
    connect(find_actions_[find_action_name_list_.indexOf("球形")],&QAction::triggered,this,&   ToolWidget::onFindSphere);

    //构造
    connect(construct_actions_[construct_action_name_list_.indexOf("点")],&QAction::triggered,this,& ToolWidget::onConstructPoint);
    connect(construct_actions_[construct_action_name_list_.indexOf("线")],&QAction::triggered,this,&ToolWidget::onConstructLine);
    connect(construct_actions_[construct_action_name_list_.indexOf("圆")],&QAction::triggered,this,&ToolWidget::onConstructCircle);
    connect(construct_actions_[construct_action_name_list_.indexOf("平面")],&QAction::triggered,this,& ToolWidget::onConstructPlane);
    connect(construct_actions_[construct_action_name_list_.indexOf("矩形")],&QAction::triggered,this,& ToolWidget::onConstructRectangle);
    connect(construct_actions_[construct_action_name_list_.indexOf("圆柱")],&QAction::triggered,this,&  ToolWidget::onConstructCylinder);
    connect(construct_actions_[construct_action_name_list_.indexOf("圆锥")],&QAction::triggered,this,&  ToolWidget::onConstructCone);
    connect(construct_actions_[construct_action_name_list_.indexOf("球形")],&QAction::triggered,this,&  ToolWidget::onConstructSphere);
    connect(construct_actions_[construct_action_name_list_.indexOf("距离")],&QAction::triggered,this,&  ToolWidget::onConstructDistance);

    //保存
    connect(save_actions_[save_action_name_list_.indexOf("excel")],&QAction::triggered,this,&  ToolWidget::onSaveExcel);
    connect(save_actions_[save_action_name_list_.indexOf("word")],&QAction::triggered,this,&  ToolWidget::onSaveWord);
    connect(save_actions_[save_action_name_list_.indexOf("txt")],&QAction::triggered,this,&  ToolWidget::onSaveTxt);
    connect(save_actions_[save_action_name_list_.indexOf("pdf")],&QAction::triggered,this,&  ToolWidget::onSavePdf);
    connect(save_actions_[save_action_name_list_.indexOf("image")],&QAction::triggered,this,&  ToolWidget::onSaveImage);

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
//Save
//从列表中提取数据转换为 QList<QList<QString>> 类型的二维列表
void   ExtractData(QVector<CEntity *>& entitylist,QList<QList<QString>>& dataAll){
    for (int i = 0; i < entitylist.size(); i++) {
        QList<QString> inList;
        CEntity* entity=entitylist[i];
        if(entity->GetUniqueType()==enPoint){
            CPoint* point=(CPoint*)entity;
            CPosition position=point->GetPt();
            inList<<"点";
            inList<<point->m_strCName;
            inList<<"坐标:("+QString::number(position.x, 'f', 6)+","+QString::number(position.y, 'f', 6)+","+QString::number(position.z, 'f', 6)+")";
        }else if(entity->GetUniqueType()==enCircle){
            CCircle* circle=(CCircle*)entity;
            CPosition position=circle->getCenter();
            inList<<"圆"<<circle->m_strAutoName;
            inList<<"中心:("+QString::number(position.x, 'f', 6)+","+QString::number(position.y, 'f', 6)+","+QString::number(position.z, 'f', 6)+")";
            inList<<"直径D:"+QString::number(circle->getDiameter(),'f',6);

        }else if(entity->GetUniqueType()==enSphere){
            CSphere* sphere=(CSphere*)entity;
            CPosition position=sphere->getCenter();
            inList<<"球"<<sphere->m_strAutoName;
            inList<<"中心:("+QString::number(position.x, 'f', 6)+","+QString::number(position.y, 'f', 6)+","+QString::number(position.z, 'f', 6)+")";
            inList<<"直径D:"+QString::number(sphere->getDiameter(),'f',6);
        }else if(entity->GetUniqueType()==enPlane){
            CPlane* plane=(CPlane*)entity;
            CPosition position=plane->getCenter();
            QVector4D normal,dir_long_edge;
            normal=plane->getNormal();
            dir_long_edge=plane->getDir_long_edge();
            inList<<"平面";
            inList<<plane->m_strAutoName;
            inList<<"中心:("+QString::number(position.x, 'f', 6)+","+QString::number(position.y, 'f', 6)+","+QString::number(position.z, 'f', 6)+")";
            inList<<"法线:("+QString::number(normal.x(), 'f', 6)+","+QString::number(normal.y(), 'f', 6)+","+QString::number(normal.z(), 'f', 6)+")";
            inList<<"边向量:("+QString::number(dir_long_edge.x(), 'f', 6)+","+QString::number(dir_long_edge.y(), 'f', 6)+","+QString::number(dir_long_edge.z(), 'f', 6)+")";
            inList<<"长:"+QString::number(plane->getLength(), 'f', 6)<<" 宽:"+QString::number(plane->getWidth(), 'f', 6);

        }else if(entity->GetUniqueType()==enCone){
            CCone* cone=(CCone*)entity;

            CPosition position=cone->getVertex();
            QVector4D axis;
            axis=cone->getAxis();
            inList<<"圆锥";
            inList<<cone->m_strAutoName;
            inList<<"顶点:("+QString::number(position.x, 'f', 6)+","+QString::number(position.y, 'f', 6)+","+QString::number(position.z, 'f', 6)+")";
            inList<<"轴线向量:("+QString::number(axis.x(), 'f', 6)+","+QString::number(axis.y(), 'f', 6)+","+QString::number(axis.z(), 'f', 6)+")";
            inList<<"高:"+QString::number(cone->getHeight(), 'f', 6)<<" 弧度:"+QString::number(cone->getRadian(), 'f', 6)<<" 圆锥高:"+QString::number(cone->getCone_height(), 'f', 6);
        }
        else if(entity->GetUniqueType()==enCylinder){
            CCylinder* cylinder=(CCylinder*)entity;
            CPosition position=cylinder->getBtm_center();
            QVector4D axis;
            axis=cylinder->getAxis();
            inList<<"圆柱";
            inList<<cylinder->m_strAutoName;
            inList<<"底面中心:("+QString::number(position.x, 'f', 6)+","+QString::number(position.y, 'f', 6)+","+QString::number(position.z, 'f', 6)+")";
            inList<<"轴线向量:("+QString::number(axis.x(), 'f', 6)+","+QString::number(axis.y(), 'f', 6)+","+QString::number(axis.z(), 'f', 6)+")";
            inList<<"高:"+QString::number(cylinder->getHeight(),'f',6)<<" 直径:"+QString::number(cylinder->getDiameter(),'f',6);

        }else if(entity->GetUniqueType()==enLine){
            CLine* line=(CLine*)entity;
            CPosition position1,position2;
            position1=line->getPosition1();
            position2=line->getPosition2();
            inList<<"线";
            inList<<line->m_strCName;
            inList<<"起点：("+QString::number(position1.x, 'f', 6)+","+QString::number(position1.y, 'f', 6)+","+QString::number(position1.z, 'f', 6)+")";
            inList<<"终点：("+QString::number(position2.x, 'f', 6)+","+QString::number(position2.y, 'f', 6)+","+QString::number(position2.z, 'f', 6)+")";
        }else if(entity->GetUniqueType()==enDistance){
            CDistance* Distance=(CDistance*) entity;
            inList<<"距离";
            inList<<Distance->m_strCName;
            inList<<QString::number(Distance->getdistance(),'f',6);
            inList<<"上公差"+QString::number(Distance->getUptolerance(),'f',6);
            inList<<"下公差"+QString::number(Distance->getUndertolerance(),'f',6);
        }else if(entity->GetUniqueType()==enPointCloud){
            CPointCloud* PointCloud=(CPointCloud*) entity;
            inList<<"点云";
            inList<<PointCloud->m_strCName;

        }
        dataAll.append(inList);
    }
}
void   ToolWidget::onSavePdf(){

    QString path = QFileDialog::getSaveFileName(nullptr, QString("Save As"), "请输入文件名", QString("Pdf(*.pdf)"));
    if (path.isEmpty()){
        return ;
    }
    auto& entitylist = m_pMainWin->m_EntityListMgr->getEntityList();
    QList<QList<QString>> dataAll;
    QList<QString> header;
    header<<"类型"<<"名称"<<"数据1"<<"数据2"<<"数据3"<<"数据4"<<"数据5";
    dataAll.append(header);
    ExtractData(entitylist,dataAll);

    // int row = 5, col = 3;
    // QList<QList<QString>> values;
    // for (int i = 0; i < row; i++) {
    //     QList<QString> inLst;
    //     for (int j = 0; j < col; j++) {
    //         inLst.append(QString::number(i) + QString::number(j));
    //     }
    //     values.append(inLst);
    // }

    if (QFileInfo(path).suffix().isEmpty())
        path.append(".pdf");

    // 只读和追加的方式打开文件
    QFile pdfFile(path);
    if (!pdfFile.open(QIODevice::WriteOnly | QIODevice::Append))
        return;

    QPdfWriter *m_pdfWriter = new QPdfWriter(&pdfFile);
    m_pdfWriter->setPageSize(QPageSize::A4);
    m_pdfWriter->setResolution(QPrinter::ScreenResolution);


    //添加标题
    //添加标题
    QString html;
    html.append("<h1 style='text-align:center;'>3dins</h1><br />");
    // QString html = addHtmlTable("T1主标题", "T1主标题", row, col, values);
    //m_html.append(html);
    // 表格主标题T1
    html.append("<table border='0.5' cellspacing='0' cellpadding='3' width:100%>");
    html.append(QString("<tr><td align='center' style='vertical-align:middle;font-weight:bold;' colspan='%1'>").arg(7));
    html.append("entitylist数据输出");
    html.append("</td></tr>");

    // 表格主标题T2
    //html.append(QString("<tr><td align='left' style='vertical-align:middle;font-weight:bold;' colspan='%1'>").arg(col));
    //html.append("T2主标题");
    //html.append("</td></tr>");

    // 添加表格数值 字段/字段值
    // 遍历表格的每个单元格，将数据插入到表格中
    for (int i = 0; i < entitylist.size()+1; ++i) {
        html.append("<tr>");
        for (int j = 0; j < dataAll[i].size(); ++j) {
            // 表头用<th></th>,有加粗效果
            //html.append(QString("<th valign='center' style='vertical-align:middle;font-size:100px;'>"));

            // 设置单元格的文本
            html.append(QString("<td valign='center' style='vertical-align:middle;font-size:100px;'>"));
            html.append(dataAll[i][j] + "</td>");
        }
        html.append("</tr>");
    }
    html.append("</table><br /><br />");

    //加入图片
    //QPainter painter;
    //painter.begin(m_pdfWriter);
    //QPixmap pixmap("./qtLogo.png");
    //painter.scale(10, 10);   //放大10倍
    //painter.drawPixmap(0, 0, pixmap);
    //painter.end();

    QTextDocument textDocument;
    textDocument.setHtml(html);
    textDocument.print(m_pdfWriter);
    textDocument.end();
     QMessageBox::information(nullptr, "提示", "保存成功");

    pdfFile.close();
}
void   ToolWidget::onSaveExcel(){

    QString filePath = QFileDialog::getSaveFileName(nullptr, QString("Save As"), "请输入文件名", QString("Excel(*.xlsx *.xls)"));
    if (filePath.isEmpty()){
        return ;
    }

    QStringList headers;
    headers << "类型" << "名称" << "数据1" << "数据2" << "数据3"<<"数据4"<<"数据5" ;
    int col = headers.size();
    QList<QList<QString>> dataAll;
    auto& entitylist = m_pMainWin->m_EntityListMgr->getEntityList();
    //dataAll.append(headers);
    ExtractData(entitylist,dataAll);

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
    QMessageBox::information(nullptr, "提示", "保存成功");
}

void ToolWidget::onSaveTxt(){
    QString filePath = QFileDialog::getSaveFileName(nullptr, QString("Save As"), "请输入文件名", QString("txt(*.txt )"));  
    if (filePath.isEmpty()){
        return ;
    }

    if (QFileInfo(filePath).suffix().isEmpty())
        filePath.append(".txt");

    // 创建 QFile 对象
    QFile file(filePath);

    // 打开文件以进行写入
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        qWarning() << "无法打开文件:" << file.errorString();
        return;
    }

    // 创建 QTextStream 对象
    QTextStream out(&file);
    auto& entitylist = m_pMainWin->m_EntityListMgr->getEntityList();
    // 写入内容

    for(int i=0;i<entitylist.size();i++){
        CEntity* entity=entitylist[i];
        if(entity->GetUniqueType()==enPoint){
            CPoint * point=(CPoint*)entity;
            CPosition position =point->GetPt();
            out<<"类型：点 名称:"<<point->m_strAutoName<<Qt::endl;
            out<<"坐标:("<<position.x<<","<<position.y<<","<<position.z<<",)"<<Qt::endl;
            out<<Qt::endl;
        }else if(entity->GetUniqueType()==enCircle){
            CCircle* circle=(CCircle*)entity;
            CPosition position=circle->getCenter();
            out<<"类型：圆 名称:"<<circle->m_strAutoName<<Qt::endl;
            out<<"中心:("<<position.x<<","<<position.y<<","<<position.z<<",)"<<Qt::endl;
            out<<"直径D:"<<circle->getDiameter();
            out<<Qt::endl;
            out<<Qt::endl;

        }else if(entity->GetUniqueType()==enSphere){
            CSphere* sphere=(CSphere*)entity;
            CPosition position=sphere->getCenter();
            out<<"类型：球 名称:"<<sphere->m_strAutoName<<Qt::endl;
            out<<"中心:("<<position.x<<","<<position.y<<","<<position.z<<",)"<<Qt::endl;
            out<<"直径D:"<<sphere->getDiameter();
            out<<Qt::endl;
             out<<Qt::endl;

        }else if(entity->GetUniqueType()==enPlane){
            CPlane* plane=(CPlane*)entity;
            CPosition position=plane->getCenter();
            QVector4D normal,dir_long_edge;
            normal=plane->getNormal();
            dir_long_edge=plane->getDir_long_edge();
             out<<"类型：平面 名称:"<<plane->m_strAutoName<<Qt::endl;
             out<<"中心:("<<position.x<<","<<position.y<<","<<position.z<<",)"<<Qt::endl;
             out<<"法线:("<<normal.x()<<","<<normal.y()<<","<<normal.z()<<",)"<<Qt::endl;
             out<<"边向量:("<<dir_long_edge.x()<<","<<dir_long_edge.y()<<","<<dir_long_edge.z()<<",)"<<Qt::endl;
             out<<"长:"<<plane->getLength()<<" 宽:"<<plane->getWidth()<<Qt::endl;
              out<<Qt::endl;

        }else if(entity->GetUniqueType()==enCone){
            CCone* cone=(CCone*)entity;

            CPosition position=cone->getVertex();
            QVector4D axis;
            axis=cone->getAxis();
            out<<"类型：圆锥 名称:"<<cone->m_strAutoName<<Qt::endl;
            out<<"顶点:("<<position.x<<","<<position.y<<","<<position.z<<",)"<<Qt::endl;
            out<<"轴线向量:("<<axis.x()<<","<<axis.y()<<","<<axis.z()<<",)"<<Qt::endl;
            out<<"高:"<<cone->getHeight()<<" 弧度:"<<cone->getRadian()<<" 圆锥高:"<<cone->getCone_height()<<Qt::endl;
          out<<Qt::endl;

        }else if(entity->GetUniqueType()==enCylinder){
            CCylinder* cylinder=(CCylinder*)entity;
            CPosition position=cylinder->getBtm_center();
            QVector4D axis;
            axis=cylinder->getAxis();
            out<<"类型：圆柱 名称:"<<cylinder->m_strAutoName<<Qt::endl;
            out<<"底面中心:("<<position.x<<","<<position.y<<","<<position.z<<",)"<<Qt::endl;
            out<<"轴线向量:("<<axis.x()<<","<<axis.y()<<","<<axis.z()<<",)"<<Qt::endl;
            out<<"高:"<<cylinder->getHeight()<<" 直径:"<<cylinder->getDiameter()<<Qt::endl;
             out<<Qt::endl;

        }else if(entity->GetUniqueType()==enLine){
            CLine* line=(CLine*)entity;
            CPosition position1,position2;
            position1=line->getPosition1();
            position2=line->getPosition2();
            out<<"类型：线 名称:"<<line->m_strAutoName<<Qt::endl;
             out<<"起点:("<<position1.x<<","<<position1.y<<","<<position1.z<<",)"<<Qt::endl;
             out<<"终点:("<<position2.x<<","<<position2.y<<","<<position2.z<<",)"<<Qt::endl;
              out<<Qt::endl;

        }else if(entity->GetUniqueType()==enDistance){
            CDistance* Distance=(CDistance*) entity;
            out<<"类型：距离 名称:"<<Distance->m_strCName<<Qt::endl;
            out<<"大小:"<<Distance->getdistance()<<" 上公差："<<Distance->getUptolerance()<<" 下公差:"<<Distance->getUndertolerance();
            out<<Qt::endl;
             out<<Qt::endl;
        }else if(entity->GetUniqueType()==enPointCloud){
            CPointCloud* PointCloud=(CPointCloud*) entity;
            out<<"类型：距离 名称:"<<PointCloud->m_strCName<<Qt::endl;
             out<<Qt::endl;
        }
        ;
    }
    // 关闭文件
    file.close();
    QMessageBox::information(nullptr, "提示", "保存成功");

}

void   ToolWidget::onSaveWord(){
    QString filePath = QFileDialog::getSaveFileName(nullptr, QString("Save As"), "文件名", QString("word(*.doc *.docx)"));
    if (filePath.isEmpty()){
        return ;
    }
    QStringList headers;
    headers << "类型" << "名称" << "数据1" << "数据2" << "数据3"<<"数据4"<<"数据5";
    auto& entitylist = m_pMainWin->m_EntityListMgr->getEntityList();
    int col = headers.size();
    int row = entitylist.size();
    QList<QList<QString>> dataAll;
    ExtractData(entitylist,dataAll);

    // 写入内容
    // 创建一个QTextDocument对象
    QTextDocument doc;

    QTextCharFormat formatTitle;
    formatTitle.setFontPointSize(16); // 设置字体大小
    formatTitle.setFontWeight(QFont::Bold); // 设置字体加粗

    // 创建一个QTextCursor对象
    QTextCursor cursor(&doc);
    // 标题和参数信息
    //cursor.insertHtml(QString("<a style='text-align：center; font-weight:bold; font-size:30px;'>%1</a>").arg(title));
    cursor.setCharFormat(formatTitle);
    cursor.insertText("entitylist列表输出");
    cursor.insertBlock(); // 换行

    QTextCharFormat format;
    format.setFontPointSize(10);
    format.setFontWeight(QFont::Bold);

    cursor.setCharFormat(format);
    //cursor.insertText("");

    // 插入一个表格，行头占一行
    QTextTable *table = cursor.insertTable(row + 1, col);

    //获取表格的格式
    QTextTableFormat tableFormat = table->format();
    //表格格式设置宽度
    tableFormat.setWidth(QTextLength(QTextLength::FixedLength, 800));

    //设置表格的columnWidthConstraints约束
    QVector<QTextLength> colLength = tableFormat.columnWidthConstraints();
    for (int i = 0; i < col; ++i) {
        colLength.append(QTextLength(QTextLength::FixedLength, tableFormat.width().rawValue() / col));
    }
    tableFormat.setColumnWidthConstraints(colLength);
    tableFormat.setBorder(5);
    tableFormat.setBorderBrush(Qt::black);

    QTextTableCellFormat titleFormat;
    titleFormat.setBackground(QColor("moccasin"));
    titleFormat.setFontWeight(QFont::Bold);
    // 设置表头 第一行下标为1
    for (int i = 0; i < col; ++i) {
        QTextTableCell cell = table->cellAt(0, i);
        cell.firstCursorPosition().insertText(headers[i]);
        cell.setFormat(titleFormat);
    }

    //定义单元格格式
    QTextTableCellFormat cellFormat;
    cellFormat.setBottomPadding(2);
    // 遍历表格的每个单元格，将数据插入到表格中
    for (int i = 0; i < row; ++i) {
        for (int j = 0; j < dataAll[i].size(); ++j) {
            // 将文本插入到表格中,第二行开始下标为2
            QTextTableCell cell = table->cellAt(i + 1, j);
            cell.firstCursorPosition().insertText(dataAll[i][j]);
            cell.setFormat(cellFormat);
        }
    }

    // 保存为Word文件
    QFile file(filePath);
    if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QTextStream stream(&file);
        //stream.setCodec("UTF-8");
        stream << doc.toHtml();
        file.close();
        QMessageBox::information(nullptr, "提示", "保存成功");
    }}
void   ToolWidget::onSaveImage(){
    QString imagePath = QFileDialog::getSaveFileName(nullptr, QString("Save As"), "请输入文件名", QString("Excel(*.png *.jpg)"));
    if (imagePath.isEmpty()){
        return ;
    }
    QStringList headers;
    headers << "类型" << "名称" << "数据1" << "数据2" << "数据3" << "数据4" << "数据5";
    auto& entitylist = m_pMainWin->m_EntityListMgr->getEntityList();
    QList<QList<QString>> dataAll;
    ExtractData(entitylist, dataAll);

    QTableWidget tableWidget(dataAll.size(), headers.size());
    tableWidget.setHorizontalHeaderLabels({"类型", "名称", "数据1", "数据2", "数据3", "数据4", "数据5"});

    // 设置水平表头自动调整大小
    tableWidget.horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);

    for (int i = 0; i < dataAll.size(); ++i) {
        QStringList& inlist = dataAll[i];
        for (int j = 0; j < inlist.size(); ++j) {
            tableWidget.setItem(i, j, new QTableWidgetItem(inlist[j]));
        }
    }

    // 调整表格大小以适应内容
    int totalWidth = 0;
    int totalHeight = 0;

    // 计算总宽度
    for (int i = 0; i < tableWidget.columnCount(); ++i) {
        totalWidth += tableWidget.columnWidth(i) + 10;
    }

    // 计算总高度
    for (int i = 0; i < tableWidget.rowCount(); ++i) {
        totalHeight += tableWidget.rowHeight(i) + 10;
    }

    tableWidget.setFixedSize(totalWidth, totalHeight);

    // 创建一个 QPixmap 对象以适应整个表格
    QPixmap pixmap(totalWidth, totalHeight);
    pixmap.fill(Qt::white);  // 设置背景为白色
    QPainter painter(&pixmap);

    // 将表格内容绘制到 QPixmap 上
    tableWidget.render(&painter);

    // 保存为图片
    pixmap.save(imagePath);
    QMessageBox::information(nullptr, "提示", "保存成功");
}

static void WrongWidget(QString message,QString moreMessage="空");
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
    qDebug()<<"ok here 1";
    QVector<CPosition>& positions= m_pMainWin->getChosenListMgr()->getChosenActorAxes();
    qDebug()<<"ok here 1.5";
    LineConstructor constructor;
    CLine* newLine;
    bool createLine=false;
    qDebug()<<"ok here 2";
    if(positions.size()==2){
        newLine=constructor.createLine(positions[0],positions[1]);
        addToList(newLine);
        createLine=true;
        positions.clear();
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
    qDebug()<<"ok here 4";
    m_pMainWin->NotifySubscribe();
     qDebug()<<"ok here 5";

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

    auto& entityList = m_pMainWin->m_EntityListMgr->getEntityList();
    RectangleConstructor constructor;
    CPlane* newRectangle=(CPlane*)constructor.create(entityList);
    if(newRectangle==nullptr){
        if(constructor.getWrongInformation()==PointTooMuch){
            WrongWidget("列表选中的点过多");
        }else if(constructor.getWrongInformation()==PointTooLess){
            WrongWidget("列表选中的点过少");
        }else if(constructor.getWrongInformation()==PointTooClose){
            WrongWidget("列表选中的点过近");
        }else if(constructor.getWrongInformation()==PointDontMatch){
            WrongWidget("四点无法构成矩形");
        }
        return ;
    }
    addToList(newRectangle);
    m_pMainWin->NotifySubscribe();

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

void ToolWidget:: onFindPlane(){
    QVector<CPosition>& positions= m_pMainWin->getChosenListMgr()->getChosenActorAxes();
    pcl::PointXYZRGB  point;
    if(positions.size()==0)return ;
    point.x=positions[0].x;
    point.y=positions[0].y;
    point.z=positions[0].z;
    auto cloudptr= m_pMainWin->getpWinFileMgr()->cloudptr;
    auto cloud=m_pMainWin->getPointCloudListMgr()->getTempCloud();
    if(cloudptr==nullptr){
        WrongWidget("点云指针为空");
        return ;
    }
    m_pMainWin->getPWinSetDataWidget()->setPlaneData(point,cloudptr);
    //std::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> shared_cloud(new pcl::PointCloud<pcl::PointXYZRGB>(cloud));
    //m_pMainWin->getPWinSetDataWidget()->setPlaneData(point,shared_cloud);
    // 生成点云对象并添加到entitylist
    // CPointCloud* pointCloud=new CPointCloud();
    auto planeCloud=m_pMainWin->getPWinSetDataWidget()->getPlaneCloud();
    if(planeCloud==nullptr){
        qDebug()<<"拟合平面生成错误";
        return ;
    }
    // pointCloud->setPointCloud(*m_pMainWin->getPWinSetDataWidget()->getFittingPlane());
    auto plane=m_pMainWin->getPWinSetDataWidget()->getPlane();
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

    positions.clear();
    m_pMainWin->NotifySubscribe();

}

void ToolWidget::onFindPoint(){
}
void ToolWidget::onFindLine(){}
void ToolWidget::onFindCircle(){}
void ToolWidget::onFindRectangle(){}
void ToolWidget::onFindCylinder(){}
void ToolWidget::onFindCone(){}
void ToolWidget::onFindSphere(){}
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
