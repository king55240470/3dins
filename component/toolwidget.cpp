#include"toolwidget.h"
#include"toolaction.h"
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
int getImagePaths(const QString& directory, QStringList &iconPaths, QStringList &iconNames);

ToolWidget::ToolWidget(QWidget *parent)
    : QWidget(parent) {
    QVBoxLayout *layout = new QVBoxLayout(this);


       resize(400,250);

        m_nSaveActionNum=4;
        m_nConstructActionNum=8;
        m_nFindActionNum=8;
        m_nCoordActionNum=3;

        //静态保存图片路径和名称
        save_action_iconpath_list_<<":/component/save/excel.png"<< ":/component/save/pdf.jpg"<< ":/component/save/txt.jpg"<< ":/component/save/word.jpg";
        construct_action_iconpath_list_<<":/component/construct/point.jpg"<<":/component/construct/line.jpg"<<":/component/construct/circle.jpg"<<   ":/component/construct/plan.jpg"<<  ":/component/construct/rectangle.jpg"<<":/component/construct/cylinder.jpg"<< ":/component/construct/cone.jpg"<< ":/component/construct/sphere.jpg";
        find_action_iconpath_list_<<":/component/construct/point.jpg"<<":/component/construct/line.jpg"<<":/component/construct/circle.jpg"<<   ":/component/construct/plan.jpg"<<  ":/component/construct/rectangle.jpg"<<":/component/construct/cylinder.jpg"<< ":/component/construct/cone.jpg"<< ":/component/construct/sphere.jpg";
        coord_action_iconpath_list_<<":/component/coord/create.png"<<  ":/component/coord/spin.jpg"<<":/component/coord/save.png";

        save_action_name_list_<<"excel"<< "pdf"<< "txt"<< "word";
        construct_action_name_list_<<"点"<<"线"<<"圆"<<"平面"<<"矩形"<<"圆柱"<<"圆锥"<<"球形";
        find_action_name_list_<<"点"<<"线"<<"圆"<<"平面"<<"矩形"<<"圆柱"<<"圆锥"<<"球形";;
        coord_action_name_list_<<"创建坐标系"<<"旋转坐标系"<<"保存坐标系";


        save_actions_      =new ToolAction * [m_nSaveActionNum];
        construct_actions_ =new ToolAction * [m_nConstructActionNum];
        find_actions_ =    new ToolAction * [m_nFindActionNum];
        coord_actions_     =new ToolAction * [m_nCoordActionNum];


    //创建工具栏
    int allActionNum=  m_nSaveActionNum+m_nConstructActionNum+m_nFindActionNum+m_nCoordActionNum;

    m_nToolbarNum =(allActionNum/SingalToolBarActionNum)+4;

    toolBars=new QToolBar*[m_nToolbarNum];


    for(int i=0;i<m_nToolbarNum;i++){

        toolBars[i]=new QToolBar(this);
        toolBars[i]->setIconSize(QSize(45,45));
        toolBars[i]->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

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

void ToolWidget::connectActionWithF(){
    //识别
    connect(find_actions_[find_action_name_list_.indexOf("点")],&QAction::triggered,&tool_widget::onFindPoint);
    connect(find_actions_[find_action_name_list_.indexOf("线")],&QAction::triggered,&tool_widget::onFindLine);
    connect(find_actions_[find_action_name_list_.indexOf("圆")],&QAction::triggered,&tool_widget::onFindCircle);
    connect(find_actions_[find_action_name_list_.indexOf("平面")],&QAction::triggered,&tool_widget::onFindPlan);
    connect(find_actions_[find_action_name_list_.indexOf("矩形")],&QAction::triggered,&tool_widget::onFindRectangle);
    connect(find_actions_[find_action_name_list_.indexOf("圆柱")],&QAction::triggered,&tool_widget::onFindCylinder);
    connect(find_actions_[find_action_name_list_.indexOf("圆锥")],&QAction::triggered,&tool_widget::onFindCone);
    connect(find_actions_[find_action_name_list_.indexOf("球形")],&QAction::triggered,&tool_widget::onFindSphere);

    //构造
    connect(construct_actions_[construct_action_name_list_.indexOf("点")],&QAction::triggered,&tool_widget::onConstructPoint);
    connect(construct_actions_[construct_action_name_list_.indexOf("线")],&QAction::triggered,&tool_widget::onConstructLine);
    connect(construct_actions_[construct_action_name_list_.indexOf("圆")],&QAction::triggered,&tool_widget::onConstructCircle);
    connect(construct_actions_[construct_action_name_list_.indexOf("平面")],&QAction::triggered,&tool_widget::onConstructPlan);
    connect(construct_actions_[construct_action_name_list_.indexOf("矩形")],&QAction::triggered,&tool_widget::onConstructRectangle);
    connect(construct_actions_[construct_action_name_list_.indexOf("圆柱")],&QAction::triggered,&tool_widget::onConstructCylinder);
    connect(construct_actions_[construct_action_name_list_.indexOf("圆锥")],&QAction::triggered,&tool_widget::onConstructCone);
    connect(construct_actions_[construct_action_name_list_.indexOf("球形")],&QAction::triggered,&tool_widget::onConstructSphere);

    //保存
    connect(save_actions_[save_action_name_list_.indexOf("excel")],&QAction::triggered,&tool_widget::onSaveExcel);
    connect(save_actions_[save_action_name_list_.indexOf("word")],&QAction::triggered,&tool_widget::onSaveWord);
    connect(save_actions_[save_action_name_list_.indexOf("txt")],&QAction::triggered,&tool_widget::onSaveTxt);
    connect(save_actions_[save_action_name_list_.indexOf("pdf")],&QAction::triggered,&tool_widget::onSavePdf);


    //坐标系
    connect(coord_actions_[coord_action_name_list_.indexOf("创建坐标系")],&QAction::triggered,&tool_widget::onCreateCoord);
    connect(coord_actions_[coord_action_name_list_.indexOf("旋转坐标系")],&QAction::triggered,&tool_widget::onSpinCoord);
    connect(coord_actions_[coord_action_name_list_.indexOf("保存坐标系")],&QAction::triggered,&tool_widget::onSaveCoord);

}


namespace tool_widget{
//Find
void onFindPoint(){ qDebug()<<"点击了识别点";}
void onFindLine(){  qDebug()<<"点击了识别线";}
void onFindCircle(){ qDebug()<<"点击了识别圆";}
void onFindPlan(){qDebug()<<"点击了识别平面";}
void onFindRectangle(){qDebug()<<"点击了识别矩形";}
void onFindCylinder(){qDebug()<<"点击了识别圆柱";}
void onFindCone(){qDebug()<<"点击了识别圆锥";}
void onFindSphere(){qDebug()<<"点击了识别球形";}
//Construct
void onConstructPoint(){qDebug()<<"点击了构造点";}
void onConstructLine(){qDebug()<<"点击了构造线";}
void onConstructCircle(){qDebug()<<"点击了构造圆";}
void onConstructPlan(){qDebug()<<"点击了构造平面";}
void onConstructRectangle(){qDebug()<<"点击了构造矩形";}
void onConstructCylinder(){qDebug()<<"点击了构造圆柱";}
void onConstructCone(){qDebug()<<"点击了构造圆锥";}
void onConstructSphere(){qDebug()<<"点击了构造球形";}
//Coord
void onCreateCoord(){qDebug()<<"点击了创建坐标系";}
void onSpinCoord(){qDebug()<<"点击了旋转坐标系";}
void onSaveCoord(){qDebug()<<"点击了保存坐标系";}
//Save
void onSavePdf(){ qDebug()<<"点击了pdf";}
void onSaveExcel(){ qDebug()<<"点击了excel";}
void onSaveTxt(){ qDebug()<<"点击了txt";}
void onSaveWord(){ qDebug()<<"点击了word";}
}

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
    // qDebug()<<iconPaths;
    // qDebug()<<iconNames;
    return imageCount;
}
