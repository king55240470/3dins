#include "mainwindow.h"
#include "component/datawidget.h"
#include "component/elementlistwidget.h"
#include "component/filemanagerwidget.h"
#include "component/toolwidget.h"
#include "component/vtkpresetwidget.h"
#include "component/vtkwidget.h"
#include "component/ReportWidget.h"
#include "component/LogWidget.h"
#include"component/contralwidget.h"
#include "geometry/centitytypes.h"
#include "component/presetelemwidget.h"

#include <QSettings>
#include <QLabel>
#include <QFileDialog>
#include <QMessageBox>
#include <QWidget>

class PresetElemWidget;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    LoadWidgets();
    setupUi();
    RestoreWidgets();
}

void MainWindow::setupUi(){
    //菜单栏
    bar=menuBar();
    setMenuBar(bar);
    QMenu *fileMenu=bar->addMenu("文件");
    QAction *openAction=fileMenu->addAction("打开文件");
    connect(openAction, &QAction::triggered, this, &MainWindow::openFile); // 连接打开文件的信号与槽
    //fileMenu->addAction(openAction);
    QAction *saveAction=fileMenu->addAction("保存文件");
    //connect(saveAction, &QAction::triggered, this, &MainWindow::saveFile); // 连接保存文件的信号与槽
    //fileMenu->addAction(saveAction);
    fileMenu->addSeparator();
    QAction *exitAction=fileMenu->addAction("退出");
    //connect(exitAction, SIGNAL(triggered()), this, SLOT(close())); // 连接退出的信号与槽
    //fileMenu->addAction(exitAction);

    QAction * contralAction=new QAction("控制图标");
    bar->addAction(contralAction);

    QMenu *presetMenu=bar->addMenu("预置元素");
    QAction* pointAction =presetMenu->addAction("点");
    connect(pointAction,&QAction::triggered,this,[&](){
        showPresetElemWidget(0);
    });
    QAction* lineAction =presetMenu->addAction("线");
    connect(lineAction,&QAction::triggered,this,[&](){
        showPresetElemWidget(1);
    });
    QAction* circleAction =presetMenu->addAction("圆");
    connect(circleAction,&QAction::triggered,this,[&](){
        showPresetElemWidget(2);
    });
    QAction* planeAction =presetMenu->addAction("平面");
    connect(planeAction,&QAction::triggered,this,[&](){
        showPresetElemWidget(3);
    });
    QAction* sphereAction =presetMenu->addAction("球");
    connect(sphereAction,&QAction::triggered,this,[&](){
        showPresetElemWidget(4);
    });
    QAction* cylinderAction =presetMenu->addAction("圆柱");
    connect(cylinderAction,&QAction::triggered,this,[&](){
        showPresetElemWidget(5);
    });
    QAction* coneAction =presetMenu->addAction("圆锥");
    connect(coneAction,&QAction::triggered,this,[&](){
        showPresetElemWidget(6);
    });

    //状态栏
    stbar=statusBar();
    setStatusBar(stbar);
    QLabel *label1=new QLabel("左侧状态栏",this);
    stbar->addWidget(label1);
    QLabel *label2=new QLabel("右侧状态栏",this);
    stbar->addPermanentWidget(label2);

    spMainWindow=new QSplitter(Qt::Horizontal,this);
    spLeft=new QSplitter(Qt::Vertical,spMainWindow);
    spMiddleup=new QSplitter(Qt::Vertical,spMainWindow);
    spRightup=new QSplitter(Qt::Vertical,spMainWindow);

    spLeft->addWidget(pWinElementListWidget);

    QTabWidget *mainTabWidget=new QTabWidget(spMiddleup);
    mainTabWidget->setTabPosition(QTabWidget::South);
    mainTabWidget->addTab(pWinVtkWidget,"图形");
    mainTabWidget->addTab(pWinReportWidget,"报表");
    spMiddleup->addWidget(mainTabWidget);

    spRightup->addWidget(pWinFileManagerWidget);

    spMiddledown=new QSplitter(Qt::Horizontal,spMiddleup);
    spMiddleup->addWidget(spMiddledown);

    QTabWidget *MiddledownTabWidget=new QTabWidget(spMiddleup);
    MiddledownTabWidget->setTabPosition(QTabWidget::South);
    MiddledownTabWidget->addTab(pWinDataWidget,"数据结果");
    MiddledownTabWidget->addTab(pWinLogWidget,"Log查看");
    spMiddledown->addWidget(MiddledownTabWidget);

    spMiddledown->addWidget(pWinToolWidget);

    spRightdown=new QSplitter(Qt::Horizontal,spRightup);
    spRightup->addWidget(spRightdown);
    spRightdown->addWidget(pWinVtkPresetWidget);

    spMainWindow->addWidget(spLeft);
    spMainWindow->addWidget(spMiddleup);
    spMainWindow->addWidget(spRightup);

    spMainWindow->setStyleSheet("QSplitter::handle{background-color:#cccccc}");
    spMainWindow->setHandleWidth(5);

    setCentralWidget(spMainWindow);


    ContralWidget * contralWidget=new ContralWidget(pWinToolWidget,this);
    connect(contralAction,&QAction::triggered,[=](){contralWidget->show();});

}

void MainWindow::LoadWidgets(){
    pWinDataWidget=new DataWidget(this);
    pWinElementListWidget=new ElementListWidget(this);
    pWinFileManagerWidget=new FileManagerWidget(this);
    pWinToolWidget=new ToolWidget(this);
    pWinVtkPresetWidget=new VtkPresetWidget(this);
    pWinVtkWidget=new VtkWidget(this);
    pWinReportWidget=new ReportWidget(this);
    pWinLogWidget=new LogWidget(this);
}

void MainWindow::RestoreWidgets() {
    QSettings settings("3D", "3D");
    QByteArray mainSplitterState = settings.value("spMainWindow/state").toByteArray();
    QByteArray leftSplitterState = settings.value("spLeft/state").toByteArray();
    QByteArray middleupSplitterState = settings.value("spMiddleup/state").toByteArray();
    QByteArray rightupSplitterState = settings.value("spRightup/state").toByteArray();
    QByteArray middledownSplitterState = settings.value("spMiddledown/state").toByteArray();
    QByteArray rightdownSplitterState = settings.value("spRightdown/state").toByteArray();

    spMainWindow->restoreState(mainSplitterState);
    spLeft->restoreState(leftSplitterState);
    spMiddleup->restoreState(middleupSplitterState);
    spRightup->restoreState(rightupSplitterState);
    spMiddledown->restoreState(middledownSplitterState);
    spRightdown->restoreState(rightdownSplitterState);
}

MainWindow::~MainWindow() {
    QSettings settings("3D", "3D");
    settings.setValue("spMainWindow/state", spMainWindow->saveState());
    settings.setValue("spLeft/state", spLeft->saveState());
    settings.setValue("spMiddleup/state", spMiddleup->saveState());
    settings.setValue("spRightup/state", spRightup->saveState());
    settings.setValue("spMiddledown/state", spMiddledown->saveState());
    settings.setValue("spRightdown/state", spRightdown->saveState());
}

void MainWindow::openFile(){
    // 使用QFileDialog打开文件对话框，允许多选
    QString filePath = QFileDialog::getOpenFileName(this, tr("open  file"));
    if (filePath.isEmpty()) { // 如果没有选择文件，返回
        return;
    }
    if (filePath.endsWith("stp")) {
        QFileInfo fileInfo(filePath);
        QString fileName = fileInfo.fileName();
        pWinFileManagerWidget->openModelFile(fileName,filePath);
    }else if(filePath.endsWith("stl")||filePath.endsWith("pcd")||filePath.endsWith("ply")){
        QFileInfo fileInfo(filePath);
        QString fileName = fileInfo.fileName();
        pWinFileManagerWidget->openMeasuredFile(fileName,filePath);
        }
}
void MainWindow::saveFile(){

}
/*void MainWindow::open_clicked() {
    // 打开文件对话框，允许用户选择文件
    QString fileName = QFileDialog::getOpenFileName(this, tr("open  file"), "", tr("point cloud files(*.pcd *.ply) ;; All files (*.*)"));

    if (fileName.isEmpty()) { // 如果没有选择文件，返回
        return;
    }
    // 根据文件扩展名加载相应的点云文件
    if (fileName.endsWith("ply")) {
        qDebug() << fileName; // 输出文件名
        if (pcl::io::loadPLYFile(fileName.toStdString(), *cloudptr) == -1) { // 加载 PLY 文件
            qDebug() << "Couldn't read .ply file \n"; // 输出错误信息
            return;
        }
    } else if (fileName.endsWith("pcd")) {
        qDebug() << fileName; // 输出文件名
        if (pcl::io::loadPCDFile(fileName.toStdString(), *cloudptr) == -1) { // 加载 PCD 文件
            qDebug() << "Couldn't read .pcd file \n"; // 输出错误信息
            return;
        }
    } else {
        QMessageBox::warning(this, "Warning", "Wrong format!"); // 弹出警告框
    }

    // 创建 PCLViewer 对象并设置窗口标题
    cloud_viewer.reset(new PCLViewer("Viewer")); // 创建点云查看器
    cloud_viewer->setShowFPS(true); // 显示帧率

    // 将 cloud_viewer 的渲染窗口嵌入到 QWidget 中
    auto viewerWinId = QWindow::fromWinId((WId)cloud_viewer->getRenderWindow()->GetGenericWindowId()); // 获取渲染窗口的 ID
    QWidget *widget = QWidget::createWindowContainer(viewerWinId, nullptr); // 创建窗口容器

    // 创建 QVBoxLayout 对象并将 QWidget 添加到其中
    QVBoxLayout *mainLayout = new QVBoxLayout; // 创建垂直布局
    mainLayout->addWidget(widget); // 将点云查看器的 QWidget 添加到布局中
    centralWidget()->setLayout(mainLayout); // 设置主窗口的中央小部件的布局

    // 设置颜色处理器，将点云数据添加到 cloud_viewer 中
    const std::string axis = "z"; // 定义颜色处理的轴
    pcl::visualization::PointCloudColorHandlerGenericField<PointT> color_handler(cloudptr, axis); // 创建颜色处理器
    cloud_viewer->addPointCloud(cloudptr, color_handler, "cloud"); // 将点云添加到查看器
    cloud_viewer->addPointCloud(cloudptr, "cloud"); // 再次添加点云（可能是为了不同的显示效果）
}

void MainWindow::save_clicked() {
    int return_status; // 返回状态
    QString filename = QFileDialog::getSaveFileName(this, tr("Open point cloud"), "", tr("Point cloud data (*.pcd *.ply)")); // 打开保存文件对话框

    if (cloudptr->empty()) { // 如果点云为空，返回
        return;
    } else {
        if (filename.isEmpty()) { // 如果没有选择文件名，返回
            return;
        }
        // 根据文件扩展名保存相应的点云文件
        if (filename.endsWith(".pcd", Qt::CaseInsensitive)) {
            return_status = pcl::io::savePCDFileBinary(filename.toStdString(), *cloudptr); // 保存 PCD 文件
        } else if (filename.endsWith(".ply", Qt::CaseInsensitive)) {
            return_status = pcl::io::savePLYFileBinary(filename.toStdString(), *cloudptr); // 保存 PLY 文件
        } else {
            filename.append(".ply"); // 默认保存为 PLY 文件
            return_status = pcl::io::savePLYFileBinary(filename.toStdString(), *cloudptr); // 保存 PLY 文件
        }
        if (return_status != 0) { // 检查返回状态
            PCL_ERROR("Error writing point cloud %s\n", filename.toStdString().c_str()); // 输出错误信息
            return;
        }
    }
}*/



CEntity* MainWindow::CreateEntity(int nType){
    CEntity *pTempEntity = NULL;
    switch (nType) {
    case enCircle:
        pTempEntity = new CCircle();
        pTempEntity->setEntityType(enCircle);
        break;
    case enPoint:
        pTempEntity = new CPoint();
        pTempEntity->setEntityType(enPoint);
        break;
    case enLine:
        pTempEntity = new CLine();
        pTempEntity->setEntityType(enLine);
        break;
    case enPlane:
        pTempEntity = new CPlane();
        pTempEntity->setEntityType(enPlane);
        break;
    case enSphere:
        pTempEntity = new CSphere();
        pTempEntity->setEntityType(enSphere);
        break;
    case enCylinder:
        pTempEntity = new CCylinder();
        pTempEntity->setEntityType(enCylinder);
        break;
    case enCone:
        pTempEntity = new CCone();
        pTempEntity->setEntityType(enCone);
        break;
    default:
        break;
    }
    pTempEntity->setDwAddress((uintptr_t)(pTempEntity));
    return pTempEntity;
}

void MainWindow::NotifySubscribe()
{
    m_ObjectListMgr->upadteelementlist();

    // 遍历entitylist重新绘制
}

void MainWindow::OnPresetPoint(CPosition pt){

    //qDebug()<<"clicked3.1";
    CPoint *pPoint = (CPoint *)CreateEntity(enPoint);
    pPoint->SetPosition(pt);
    pPoint->m_CreateForm = ePreset;
    pPoint->m_pRefCoord = m_pcsListMgr->m_pPcsCurrent;
    pPoint->m_pCurCoord = m_pcsListMgr->m_pPcsCurrent;
    pPoint->m_pExtCoord = m_pcsListMgr->m_pPcsCurrent;
    // pPoint->SetNominal();
    // 加入Entitylist 和 ObjectList
    m_EntityListMgr->Add(pPoint);
    m_ObjectListMgr->Add(pPoint);

    //qDebug()<<"clicked3.2";
    //NotifySubscribe();
}

// void MainWindow::OnPresetLine(CPosition ptStart, CPosition ptEnd)
// {
//     CLine *pLine = (CLine *)CreateEntity(enLine);
//     pLine->setBegin(ptStart);
//     pLine->setEnd(ptEnd);
//     pLine->m_CreateForm = ePreset;
//     pLine->m_pRefCoord = m_pcsListMgr->m_pPcsCurrent;
//     pLine->m_pCurCoord = m_pcsListMgr->m_pPcsCurrent;
//     pLine->m_pExtCoord = m_pcsListMgr->m_pPcsCurrent;
//     // pLine->SetNominal();

//     // 加入Entitylist 和 ObjectList
//     m_EntityListMgr->Add(pLine);
//     m_ObjectListMgr->Add(pLine);

//     //NotifySubscribe();
// }

// void MainWindow::OnPresetCircle(CPosition pt, double diameter)
// {
//     //CPosition pt(1,1,1);
//     //double diameter = 1.5;

//     CCircle *pCircle = (CCircle *)CreateEntity(enCircle);
//     pCircle->SetCenter(pt);
//     pCircle->SetDiameter(diameter);
//     pCircle->m_CreateForm = ePreset;
//     pCircle->m_pRefCoord = m_pcsListMgr->m_pPcsCurrent;
//     pCircle->m_pCurCoord = m_pcsListMgr->m_pPcsCurrent;
//     pCircle->m_pExtCoord = m_pcsListMgr->m_pPcsCurrent;

//     pCircle->SetNominal();

//     std::vector<QPointF> points;

//     pCircle->addToolArray((CircleGraphics*)CreateImgTool(eImgCircleGraphics,points,data));
//     // 加入Entitylist 和 ObjectList
//     m_EntityListMgr->Add(pCircle);
//     if(m_ObjectListMgr->GetInsertPos()==-1){
//         m_ObjectListMgr->Add(pCircle);
//         //pWinElementListWidget->updateInsertIndicatorPosition();

//     }else{
//         m_ObjectListMgr->Insert(pCircle, m_ObjectListMgr->m_nInsertPos);
//         m_ObjectListMgr->m_nInsertPos++;
//         //pWinElementListWidget->updateInsertIndicatorPosition();
//     }

//     //选中元素,取消其他元素选中
//     for(auto const &object:m_ObjectListMgr->m_objectList){
//         object->SetSelected(false);
//     }
//     pCircle->SetSelected(true);

//     //NotifySubscribe();
// }

// void MainWindow::OnPresetPlane(CPosition posCenter, QVector4D normal, QVector4D direction, double length, double width)
// {
//     CPlane *pPlane = (CPlane *)CreateEntity(enPlane);
//     pPlane->setCenter(posCenter);
//     pPlane->setDir_long_edge(direction);
//     pPlane->setNormal(normal);
//     pPlane->setLength(length);
//     pPlane->setWidth(width);

//     pPlane->m_CreateForm = ePreset;
//     pPlane->SetCurCoord(m_pcsListMgr->m_pPcsCurrent);
//     pPlane->SetRefCoord(m_pcsListMgr->m_pPcsCurrent);
//     pPlane->SetExtCoord(m_pcsListMgr->m_pPcsCurrent);

//     // pPlane->SetNominal();

//     // 加入Entitylist 和 ObjectList
//     m_EntityListMgr->Add(pPlane);
//     m_ObjectListMgr->Add(pPlane);

//     //NotifySubscribe();

// }

// void MainWindow::OnPresetSphere(CPosition posCenter, double diametre)
// {
//     CSphere *pSphere = (CSphere *)CreateEntity(enSphere);
//     pSphere->setCenter(posCenter);
//     pSphere->setDiameter(diametre);

//     pSphere->m_CreateForm = ePreset;
//     pSphere->SetCurCoord(m_pcsListMgr->m_pPcsCurrent);
//     pSphere->SetRefCoord(m_pcsListMgr->m_pPcsCurrent);
//     pSphere->SetExtCoord(m_pcsListMgr->m_pPcsCurrent);

//     // pSphere->SetNominal();

//     // 加入Entitylist 和 ObjectList
//     m_EntityListMgr->Add(pSphere);
//     m_ObjectListMgr->Add(pSphere);
//     qDebug()<<"add a sphere";
//     //NotifySubscribe();
// }

// void MainWindow::OnPresetCylinder(CPosition pos, QVector4D vec, double height, double diametre)
// {
//     CCylinder *pCylinder = (CCylinder *)CreateEntity(enCylinder);
//     pCylinder->setAxis(vec);
//     pCylinder->setBtm_center(pos);
//     pCylinder->setDiameter(diametre);
//     pCylinder->setHeight(height);

//     pCylinder->m_CreateForm = ePreset;
//     pCylinder->SetCurCoord(m_pcsListMgr->m_pPcsCurrent);
//     pCylinder->SetRefCoord(m_pcsListMgr->m_pPcsCurrent);
//     pCylinder->SetExtCoord(m_pcsListMgr->m_pPcsCurrent);

//     // pCylinder->SetNominal();

//     // 加入Entitylist 和 ObjectList
//     m_EntityListMgr->Add(pCylinder);
//     m_ObjectListMgr->Add(pCylinder);

//     qDebug()<<"add a Cone";
//     //NotifySubscribe();

// }


// void MainWindow::OnPresetCone(CPosition posCenter, QVector4D axis, double partH, double fullH, double angle)

// {
//     CCone *pCone = (CCone *)CreateEntity(enCone);
//     pCone->setCone_height(partH);
//     pCone->setHeight(fullH);
//     pCone->setAxis(axis);
//     pCone->setRadian(angle);
//     pCone->setVertex(posCenter);

//     pCone->m_CreateForm = ePreset;
//     pCone->SetCurCoord(m_pcsListMgr->m_pPcsCurrent);
//     pCone->SetRefCoord(m_pcsListMgr->m_pPcsCurrent);
//     pCone->SetExtCoord(m_pcsListMgr->m_pPcsCurrent);

//     // pCone->SetNominal();

//     // 加入Entitylist 和 ObjectList
//     m_EntityListMgr->Add(pCone);
//     m_ObjectListMgr->Add(pCone);

//     qDebug()<<"add a Cone";
//     //NotifySubscribe();
// }

void MainWindow::showPresetElemWidget(int index){
    pWinPresetElemWidget=new PresetElemWidget(this);
    switch(index){
    case 0:pWinPresetElemWidget->tabWidget->setCurrentIndex(0);
        break;
    case 1:pWinPresetElemWidget->tabWidget->setCurrentIndex(1);
        break;
    case 2:pWinPresetElemWidget->tabWidget->setCurrentIndex(2);
        break;
    case 3:pWinPresetElemWidget->tabWidget->setCurrentIndex(3);
        break;
    case 4:pWinPresetElemWidget->tabWidget->setCurrentIndex(4);
        break;
    case 5:pWinPresetElemWidget->tabWidget->setCurrentIndex(5);
        break;
    case 6:pWinPresetElemWidget->tabWidget->setCurrentIndex(6);
        break;
    }
    pWinPresetElemWidget->show();
}
