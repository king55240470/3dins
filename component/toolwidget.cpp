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


int getImagePaths(const QString& directory, QStringList &iconPaths, QStringList &iconNames);

ToolWidget::ToolWidget(QWidget *parent)
    : QWidget(parent) {
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
    qDebug()<<"ok3";
    qDebug()<<m_nViewAngleActionNum;
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
    //识别
    connect(find_actions_[find_action_name_list_.indexOf("点")],&QAction::triggered,&  tool_widget::onFindPoint);
    connect(find_actions_[find_action_name_list_.indexOf("线")],&QAction::triggered,&  tool_widget::onFindLine);
    connect(find_actions_[find_action_name_list_.indexOf("圆")],&QAction::triggered,&  tool_widget::onFindCircle);
    connect(find_actions_[find_action_name_list_.indexOf("平面")],&QAction::triggered,&  tool_widget::onFindPlan);
    connect(find_actions_[find_action_name_list_.indexOf("矩形")],&QAction::triggered,&  tool_widget::onFindRectangle);
    connect(find_actions_[find_action_name_list_.indexOf("圆柱")],&QAction::triggered,&  tool_widget::onFindCylinder);
    connect(find_actions_[find_action_name_list_.indexOf("圆锥")],&QAction::triggered,&  tool_widget::onFindCone);
    connect(find_actions_[find_action_name_list_.indexOf("球形")],&QAction::triggered,&  tool_widget::onFindSphere);

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
    connect(save_actions_[save_action_name_list_.indexOf("excel")],&QAction::triggered,&  tool_widget::onSaveExcel);
    connect(save_actions_[save_action_name_list_.indexOf("word")],&QAction::triggered,&  tool_widget::onSaveWord);
    connect(save_actions_[save_action_name_list_.indexOf("txt")],&QAction::triggered,&  tool_widget::onSaveTxt);
    connect(save_actions_[save_action_name_list_.indexOf("pdf")],&QAction::triggered,&  tool_widget::onSavePdf);
    connect(save_actions_[save_action_name_list_.indexOf("image")],&QAction::triggered,&  tool_widget::onSaveImage);
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
void  onFindPoint(){ qDebug()<<"点击了识别点";}
void  onFindLine(){  qDebug()<<"点击了识别线";}
void  onFindCircle(){ qDebug()<<"点击了识别圆";}
void  onFindPlan(){qDebug()<<"点击了识别平面";}
void  onFindRectangle(){qDebug()<<"点击了识别矩形";}
void  onFindCylinder(){qDebug()<<"点击了识别圆柱";}
void  onFindCone(){qDebug()<<"点击了识别圆锥";}
void   onFindSphere(){qDebug()<<"点击了识别球形";}
//Construct
void   onConstructPoint(){qDebug()<<"点击了构造点";}
void   onConstructLine(){qDebug()<<"点击了构造线";}
void   onConstructCircle(){qDebug()<<"点击了构造圆";}
void   onConstructPlane(){qDebug()<<"点击了构造平面";}
void   onConstructRectangle(){qDebug()<<"点击了构造矩形";}
void   onConstructCylinder(){qDebug()<<"点击了构造圆柱";}
void   onConstructCone(){qDebug()<<"点击了构造圆锥";}
void   onConstructSphere(){qDebug()<<"点击了构造球形";}
//Coord
void   onCreateCoord(){qDebug()<<"点击了创建坐标系";}
void   onSpinCoord(){qDebug()<<"点击了旋转坐标系";}
void   onSaveCoord(){qDebug()<<"点击了保存坐标系";}
//Save
void   onSavePdf(){
    QString path = QFileDialog::getSaveFileName(nullptr, QString("Save As"), "请输入文件名", QString("Pdf(*.pdf)"));
    if (path.isEmpty()){
        return ;
    }
    int row = 5, col = 3;
    QList<QList<QString>> values;
    for (int i = 0; i < row; i++) {
        QList<QString> inLst;
        for (int j = 0; j < col; j++) {
            inLst.append(QString::number(i) + QString::number(j));
        }
        values.append(inLst);
    }

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
    html.append("<h1 style='text-align:center;'>导出为pdf</h1><br />");
    // QString html = addHtmlTable("T1主标题", "T1主标题", row, col, values);
    //m_html.append(html);
    // 表格主标题T1
    html.append("<table border='0.5' cellspacing='0' cellpadding='3' width:100%>");
    html.append(QString("<tr><td align='center' style='vertical-align:middle;font-weight:bold;' colspan='%1'>").arg(col));
    html.append("T1主标题");
    html.append("</td></tr>");

    // 表格主标题T2
    //html.append(QString("<tr><td align='left' style='vertical-align:middle;font-weight:bold;' colspan='%1'>").arg(col));
    //html.append("T2主标题");
    //html.append("</td></tr>");

    // 添加表格数值 字段/字段值
    // 遍历表格的每个单元格，将数据插入到表格中
    for (int i = 0; i < row; ++i) {
        html.append("<tr>");
        for (int j = 0; j < col; ++j) {
            // 表头用<th></th>,有加粗效果
            //html.append(QString("<th valign='center' style='vertical-align:middle;font-size:100px;'>"));

            int index = col + 1;
            // 设置单元格的文本
            html.append(QString("<td valign='center' style='vertical-align:middle;font-size:100px;'>"));
            html.append(values[i][j] + "</td>");
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

    pdfFile.close();
}
void   onSaveExcel(){
    QString filePath = QFileDialog::getSaveFileName(nullptr, QString("Save As"), "请输入文件名", QString("Excel(*.xlsx *.xls)"));
    if (filePath.isEmpty()){
        return ;
    }
    QStringList headers;
    headers << "表头1" << "表头2" << "表头3" << "表头4" << "表头5" ;
    int col = headers.size();
    int row = 6;
    QList<QList<QString>> dataAll;
    for (int i = 0; i < row; i++) {
        QList<QString> inLst;
        for (int j = 0; j < col; j++) {
            inLst.append(QString::number(i) + QString::number(j));
        }
        dataAll.append(inLst);
    }

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
    cell->dynamicCall("SetValue(const QString&)", "excel导出示例");
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
        for (int j = 0; j < col; j++) {
            // ("Cells(int, int)", row, col)单元格的行和列从开始
            QAxObject *cell = workSheet->querySubObject("Cells(int, int)", curRow, j+1);
            cell->dynamicCall("SetValue(const QString&)", inLst[j]);
            QAxObject* border = cell->querySubObject("Borders");
            border->setProperty("Color", QColor(0, 0, 0));		 //设置单元格边框色（黑色）
        }
        curRow++;
    }

    //保存至filepath，注意一定要用QDir::toNativeSeparators将路径中的"/"转换为"\"，不然一定保存不了。
    workBook->dynamicCall("SaveAs(const QString&)", QDir::toNativeSeparators(filePath));
    workBook->dynamicCall("Close()");	//关闭工作簿
    excel.dynamicCall("Quit()");		//关闭excel
    QMessageBox::information(nullptr, "提示", "保存成功");
}
void  onSaveTxt(){
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

    // 写入内容
    out << "Hello, World!" ;
    out << "This is a line of text." ;

    // 关闭文件
    file.close();
    QMessageBox::information(nullptr, "提示", "保存成功");

}
void   onSaveWord(){
    QString filePath = QFileDialog::getSaveFileName(nullptr, QString("Save As"), "请输入文件名", QString("word(*.doc *.docx)"));
    if (filePath.isEmpty()){
        return ;
    }
    QStringList headers;
    headers << "表头1" << "表头2" << "表头3" << "表头4" << "表头5";
    int col = headers.size();
    int row = 6;
    QList<QList<QString>> dataAll;
    for (int i = 0; i < row; i++) {
        QList<QString> inLst;
        for (int j = 0; j < col; j++) {
            inLst.append(QString::number(i) + QString::number(j));
        }
        dataAll.append(inLst);
    }

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
    cursor.insertText("导出word示例");
    cursor.insertBlock(); // 换行

    QTextCharFormat format;
    format.setFontPointSize(10);
    format.setFontWeight(QFont::Bold);

    cursor.setCharFormat(format);
    cursor.insertText("提示信息，表格之前的文字描述");

    // 插入一个表格，行头占一行
    QTextTable *table = cursor.insertTable(row + 1, col);

    //获取表格的格式
    QTextTableFormat tableFormat = table->format();
    //表格格式设置宽度
    tableFormat.setWidth(QTextLength(QTextLength::FixedLength, 500));

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
        for (int j = 0; j < col; ++j) {
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
void   onSaveImage(){
    QString imagePath = QFileDialog::getSaveFileName(nullptr, QString("Save As"), "请输入文件名", QString("Excel(*.png *.jpg)"));
    if (imagePath.isEmpty()){
        return ;
    }

    QTableWidget TableWidget(4, 3); // 4 行 3 列
    TableWidget.setHorizontalHeaderLabels({"Column 1", "Column 2", "Column 3"});
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 3; ++col) {
            TableWidget.setItem(row, col, new QTableWidgetItem(QString("Item %1-%2").arg(row).arg(col)));
        }
    }
    QTableWidget * tableWidget=&TableWidget;
    // 调整表格大小以适应内容
    int totalWidth = 0;
    int totalHeight = 0;

    // 计算总宽度
    for (int i = 0; i < tableWidget->columnCount(); ++i) {
        totalWidth += tableWidget->columnWidth(i)+10;
    }

    // 计算总高度
    for (int i = 0; i < tableWidget->rowCount(); ++i) {
        totalHeight += tableWidget->rowHeight(i)+10;
    }
    tableWidget->setFixedSize(totalWidth,totalHeight);
    // 创建一个 QPixmap 对象以适应整个表格
    QPixmap pixmap(totalWidth, totalHeight);
    pixmap.fill(Qt::white);  // 设置背景为白色
    QPainter painter(&pixmap);

    // 将表格内容绘制到 QPixmap 上
    tableWidget->render(&painter);

    // 保存为图片
    pixmap.save(imagePath);
}
}
static void WrongWidget(QString message);
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

    m_pMainWin->NotifySubscribe();

}
void ToolWidget::onConstructPoint(){

    QVector<CPosition>& positions= m_pMainWin->getChosenListMgr()->getChosenCEntityList();;
    PointConstructor constructor;
    CPoint* newPoint;
    bool createPoint=false;
    qDebug()<<positions.size();
    for(int i=0;i<positions.size();i++){
        qDebug()<<positions[i].x<<positions[i].y<<positions[i].z;
    }
    if(positions.size()!=0){
        createPoint=true;
        for(int i=0;i<positions.size();i++){
            newPoint=constructor.createPoint(positions[i]);
            if(newPoint!=nullptr){
                addToList(newPoint);
            }
        }
    }

    auto& entityList = m_pMainWin->m_EntityListMgr->getEntityList();
    newPoint=(CPoint*)constructor.create(entityList);
    positions=constructor.getPositions();
    if(newPoint!=nullptr){
        addToList(newPoint);
        for(int i=0;i<positions.size();i++){
            if(i!=0){
                newPoint=constructor.createPoint(positions[i]);
            }
            addToList(newPoint);
        }
    }

    if(!createPoint&&newPoint==nullptr){
        WrongWidget("构造点失败");
        return ;
    }
    m_pMainWin->NotifySubscribe();
}

void ToolWidget::onConstructLine(){
    auto& entityList = m_pMainWin->m_EntityListMgr->getEntityList();
    LineConstructor constructor;
    CLine* newLine=(CLine*)constructor.create(entityList);
    if(newLine==nullptr){
        WrongWidget("构造线失败");
        return ;
    }

    addToList(newLine);

}
static void WrongWidget(QString message){
    QMessageBox msgBox;
    msgBox.setWindowTitle("错误");
    msgBox.setText(message);
    msgBox.setIcon(QMessageBox::Critical); // 设置对话框图标为错误
    msgBox.setStandardButtons(QMessageBox::Ok); // 只显示“确定”按钮
    msgBox.exec(); // 显示对话框
}


void ToolWidget::onConstructCircle(){
    auto& entityList = m_pMainWin->m_EntityListMgr->getEntityList();
    CircleConstructor constructor;
    CCircle* newCircle=(CCircle*)constructor.create(entityList);
    if(newCircle==nullptr){
        WrongWidget("构造圆失败");
        return ;
    }
    addToList(newCircle);
}
void ToolWidget::onConstructPlane(){
    auto& entityList = m_pMainWin->m_EntityListMgr->getEntityList();
    PlaneConstructor constructor;
    CPlane* newPlane=(CPlane*)constructor.create(entityList);
    if(newPlane==nullptr){
        WrongWidget("构造平面失败");
        return ;
    }
    addToList(newPlane);
}
void ToolWidget::onConstructRectangle(){

    auto& entityList = m_pMainWin->m_EntityListMgr->getEntityList();
    RectangleConstructor constructor;
    CPlane* newRectangle=(CPlane*)constructor.create(entityList);
    if(newRectangle==nullptr){
        WrongWidget("构造矩形失败");
        return ;
    }
    addToList(newRectangle);
}

void ToolWidget::onConstructSphere(){

    auto& entityList = m_pMainWin->m_EntityListMgr->getEntityList();
    SphereConstructor constructor;
    CSphere* newSphere=(CSphere*)constructor.create(entityList);
    if(newSphere==nullptr){
        WrongWidget("构造球失败");
        return ;
    }
    addToList(newSphere);
}

void ToolWidget::onConstructCone(){
    auto& entityList = m_pMainWin->m_EntityListMgr->getEntityList();
    ConeConstructor constructor;
    CCone* newCone=(CCone*)constructor.create(entityList);
    if(newCone==nullptr){
        WrongWidget("构造球失败");
        return ;
    }
    addToList(newCone);
}
void ToolWidget::onConstructCylinder(){
    auto& entityList = m_pMainWin->m_EntityListMgr->getEntityList();
    CylinderConstructor constructor;
    CCylinder* newCylinder=(CCylinder*)constructor.create(entityList);
    if(newCylinder==nullptr){
        WrongWidget("构造圆柱失败");
        return ;
    }
    addToList(newCylinder);
}
void ToolWidget::onConstructDistance(){

    auto& entityList = m_pMainWin->m_EntityListMgr->getEntityList();
    DistanceConstructor constructor;
    //constructor.createTolerance(0,0);
    CDistance* newDistance=(CDistance*)constructor.create(entityList);
    newDistance->setUptolerance(0);
    newDistance->setUndertolerance(0);
    if(newDistance==nullptr){
        WrongWidget("构造距离失败");
        return ;
    }
    addToList(newDistance);
}

void ToolWidget::updateele(){

    ElementListWidget* p_elementListwidget= m_pMainWin->getPWinElementListWidget();

    QList<QTreeWidgetItem*> selectedItems = p_elementListwidget->getSelectedItems();
    QVector<CObject*> eleobjlist=p_elementListwidget->getEleobjlist();

    if (!selectedItems.isEmpty()) {
        m_point_index.clear();
        m_selected_points.clear();
        m_selected_plane.clear();
        int *index=new int[selectedItems.size()];
        int *entityindex=new int[selectedItems.size()];
        int count_index=0;
        int count_entityindex=0;
        qDebug()<<"被选中的元素有："<<selectedItems.size();
        for(QTreeWidgetItem *selectedItem:selectedItems)
        {

            CObject *obj = selectedItem->data(0, Qt::UserRole).value<CObject*>();
            for(int i=0;i<m_pMainWin->getObjectListMgr()->getObjectList().size();i++){
                if(m_pMainWin->getObjectListMgr()->getObjectList()[i]==obj){
                    index[count_index]=i;
                    count_index++;
                }
            }

            for(int i=0;i<eleobjlist.size();i++){
                if(eleobjlist[i]==obj){
                    entityindex[count_entityindex]=i;
                    count_entityindex++;
                }
            }

        }
        auto& objectList = m_pMainWin->m_ObjectListMgr->getObjectList();
        auto& entityList = m_pMainWin->m_EntityListMgr->getEntityList();

        for(int i=0;i<count_entityindex;i++){
            if(entityList[entityindex[i]]->GetUniqueType()==enPoint){
                m_point_index.push_back(index[i]);
                m_selected_points.push_back((CPoint*)entityList[entityindex[i]]);
            }
        }
        for(int i=0;i<count_entityindex;i++){
            if(entityList[entityindex[i]]->GetUniqueType()==enPlane){
                m_plane_index.push_back(index[i]);
                m_selected_plane.push_back((CPlane*)entityList[entityindex[i]]);
            }
        }
        delete []index;
        delete []entityindex;
    }
}
void ToolWidget::NotifySubscribe(){
    qDebug()<<"ToolWidget::NotifySubscribe()";
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

QVector<CEntity*>& ToolWidget::getConstructEntityList(){
    return constructEntityList;
}
