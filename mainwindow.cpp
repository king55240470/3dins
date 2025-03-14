#include "mainwindow.h"
#include "component/datawidget.h"
#include "component/elementlistwidget.h"
#include "component/filemanagerwidget.h"
#include "component/toolwidget.h"
#include "vtkwindow/vtkpresetwidget.h"
#include "vtkwindow/vtkwidget.h"
#include "component/ReportWidget.h"
#include "component/LogWidget.h"
#include"component/contralwidget.h"
#include "geometry/centitytypes.h"
#include "component/presetelemwidget.h"
#include "manager/filemgr.h"
#include "pointfitting/setdatawidget.h"

#include <QSettings>
#include <QLabel>
#include <QFileDialog>
#include <QMessageBox>
#include <QWidget>
#include "manager/cpcsmgr.h"
#include <QListWidget>
#include <QIODevice>
#include <QShortcut>
#include <QStackedWidget>

class PresetElemWidget;

double MainWindow::ActorColor[3] = {0.5, 0.5, 0.5};
double MainWindow::HighLightColor[3] = {1, 1, 0};
double MainWindow::InfoTextColor[3] = {0.9, 0.9, 0.9};

// 控制图形渲染的粗细
double MainWindow::ActorPointSize = 4;
double MainWindow::ActorLineWidth = 3;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    LoadWidgets();
    setupUi();
    RestoreWidgets();
    loadManager();
    LoadSetDataWidget();
    //filechange();
    m_nRelyOnWhichCs=csRef;
    SetUpTheme();
    modelCloudExist=false;
}

void MainWindow::setupUi(){
    //菜单栏
    bar=menuBar();
    setMenuBar(bar);
    QIcon openFile(":/style/openfile.png");
    QIcon saveFile(":/style/savefile.png");
    QIcon exitIcon(":/style/exit.png");
    QMenu *fileMenu=bar->addMenu("文件(F)");
    QAction *openAction=fileMenu->addAction("打开文件");
    openAction->setIcon(openFile);
    openAction->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_O));
    openAction->setShortcutContext(Qt::ApplicationShortcut);
    connect(openAction, &QAction::triggered, this, &MainWindow::openFile); // 连接打开文件的信号与槽
    QAction *saveAction=fileMenu->addAction("保存文件");
    saveAction->setIcon(saveFile);
    saveAction->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_S));
    saveAction->setShortcutContext(Qt::ApplicationShortcut);
    connect(saveAction, &QAction::triggered, this, &MainWindow::saveFile); // 连接保存文件的信号与槽
    fileMenu->addSeparator();
    QAction *exitAction=fileMenu->addAction("退出");
    exitAction->setIcon(exitIcon);
    exitAction->setShortcut(QKeySequence(Qt::SHIFT | Qt::Key_Escape));
    connect(exitAction, SIGNAL(triggered()), this, SLOT(close())); // 连接退出的信号与槽
    // 创建一个隐藏的 QAction，用于触发菜单
    QAction* showMenuAction = new QAction();
    showMenuAction->setShortcut(QKeySequence(Qt::Key_F)); // 设置快捷键为 F
    connect(showMenuAction, &QAction::triggered, this, [fileMenu,this]() {
        QPoint globalPos = mapToGlobal(QPoint(0, 22));
        fileMenu->popup(globalPos);
    });
    addAction(showMenuAction);

    QAction* contralAction=new QAction("控制图标(Z)");
    contralAction->setShortcut(QKeySequence(Qt::Key_Z));
    bar->addAction(contralAction);

    QAction* actorAdjust = new QAction("渲染调整(A)");
    actorAdjust->setShortcut(QKeySequence(Qt::Key_A));
    bar->addAction(actorAdjust);

    QMenu* windowMenu = bar->addMenu("窗口(W)");
    QAction* showWinMenu = new QAction();
    showWinMenu->setShortcut(QKeySequence(Qt::Key_W));
    connect(showWinMenu, &QAction::triggered, this, [this,windowMenu]() {
        QPoint globalPos = mapToGlobal(QPoint(250, 22));
        windowMenu->exec(globalPos);
    });
    addAction(showWinMenu);
    QIcon mainWin(":/component/eye/main_win.png");
    QIcon reportWin(":/component/eye/report_win.png");
    QAction* showVtkWin = windowMenu->addAction("主窗口");
    showVtkWin->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_W));
    showVtkWin->setShortcutContext(Qt::ApplicationShortcut);
    showVtkWin->setIcon(mainWin);

    QAction* showReportWin = windowMenu->addAction("报表窗口");
    showReportWin->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_R));
    showReportWin->setShortcutContext(Qt::ApplicationShortcut);
    showReportWin->setIcon(reportWin);

    QMenu *presetMenu=bar->addMenu("预置(P)");
    QAction* showPresetMenu = new QAction();
    showPresetMenu->setShortcut(QKeySequence(Qt::Key_P));
    connect(showPresetMenu, &QAction::triggered, this, [this,presetMenu]() {
        QPoint globalPos = mapToGlobal(QPoint(310, 22));
        presetMenu->exec(globalPos);
    });
    addAction(showPresetMenu);
    // 创建图标，用于构造、拟合、预置等功能的指示
    QIcon pointIcon(":/component/construct/point.jpg");
    QIcon lineIcon(":/component/construct/line.jpg");
    QIcon circleIcon(":/component/construct/circle.jpg");
    QIcon planeIcon(":/component/construct/plan.jpg");
    QIcon sphereIcon(":/component/construct/sphere.jpg");
    QIcon cylinderIcon(":/component/construct/cylinder.jpg");
    QIcon coneIcon(":/component/construct/cone.jpg");
    QIcon boxIcon(":/component/viewangle/isometric.png");
    QIcon distanceIcon(":/component/construct/distance.png");
    QIcon rectangleIcon(":/component/construct/rectangle.jpg");
    QIcon cloudIcon(":/component/construct/pointCloud.png");
    QIcon angleIcon(":/component/construct/angle.png");

    QAction* pointAction =presetMenu->addAction("点");
    pointAction->setIcon(pointIcon);
    connect(pointAction,&QAction::triggered,this,[&](){
        showPresetElemWidget(0);
    });
    QAction* lineAction =presetMenu->addAction("线");
    lineAction->setIcon(lineIcon);
    connect(lineAction,&QAction::triggered,this,[&](){
        showPresetElemWidget(1);
    });
    QAction* circleAction =presetMenu->addAction("圆");
    circleAction->setIcon(circleIcon);
    connect(circleAction,&QAction::triggered,this,[&](){
        showPresetElemWidget(2);
    });
    QAction* planeAction =presetMenu->addAction("平面");
    planeAction->setIcon(planeIcon);
    connect(planeAction,&QAction::triggered,this,[&](){
        showPresetElemWidget(3);
    });
    QAction* sphereAction =presetMenu->addAction("球");
    sphereAction->setIcon(sphereIcon);
    connect(sphereAction,&QAction::triggered,this,[&](){
        showPresetElemWidget(4);
    });
    QAction* cylinderAction =presetMenu->addAction("圆柱");
    cylinderAction->setIcon(cylinderIcon);
    connect(cylinderAction,&QAction::triggered,this,[&](){
        showPresetElemWidget(5);
    });
    QAction* coneAction =presetMenu->addAction("圆锥");
    coneAction->setIcon(coneIcon);
    connect(coneAction,&QAction::triggered,this,[&](){
        showPresetElemWidget(6);
    });
    QAction* boxAction =presetMenu->addAction("长方体");
    boxAction->setIcon(boxIcon);
    connect(boxAction,&QAction::triggered,this,[&](){
        showPresetElemWidget(7);
    });

    // QMenu* dateOperation=bar->addMenu("配置");
    // QAction* storeAction=dateOperation->addAction("数据记录");
    // connect(storeAction,&QAction::triggered,this,&MainWindow::SaveIniFile);

    // QMenu *fittingMenu=bar->addMenu("拟合参数设置");
    // QAction* fittingPlaneAction=fittingMenu->addAction("拟合平面");
    // connect(fittingPlaneAction, &QAction::triggered, this, &setDataWidget::setPlaneData);
    // QAction* fittingCylinderAction=fittingMenu->addAction("拟合圆柱");
    // connect(fittingCylinderAction, &QAction::triggered, this, &setDataWidget::setCylinderData);
    // QAction* fittingConeAction=fittingMenu->addAction("拟合圆锥");
    // connect(fittingConeAction, &QAction::triggered, this, &setDataWidget::setConeData);
    // QAction* fittingSphereAction=fittingMenu->addAction("拟合球");
    // connect(fittingSphereAction, &QAction::triggered, this, &setDataWidget::setSphereData);
    // QAction* fittingLineAction=fittingMenu->addAction("拟合直线");
    // connect(fittingLineAction, &QAction::triggered, this, &setDataWidget::setLineData);

    QMenu* constructorMenu = bar->addMenu("构造(C)");
    QAction* showConsMenu = new QAction();
    showConsMenu->setShortcut(QKeySequence(Qt::Key_C));
    connect(showConsMenu, &QAction::triggered, this, [this,constructorMenu]() {
        QPoint globalPos = mapToGlobal(QPoint(370, 22));
        constructorMenu->exec(globalPos);
    });
    addAction(showConsMenu);
    QAction* constructPoint = constructorMenu->addAction("点");
    QAction* constructLine = constructorMenu->addAction("线");
    QAction* constructPlane = constructorMenu->addAction("平面");
    QAction* constructCircle = constructorMenu->addAction("圆");
    QAction* constructSphere = constructorMenu->addAction("球体");
    QAction* constructRect= constructorMenu->addAction("矩形");
    QAction* constructCylinder = constructorMenu->addAction("圆柱");
    QAction* constructCone = constructorMenu->addAction("圆锥");
    QAction* constructDis= constructorMenu->addAction("距离");
    QAction* constructCloud = constructorMenu->addAction("点云切割");
    QAction* constructAngle = constructorMenu->addAction("角度");
    // 插入图片
    QIcon point_construct(":/component/construct/point_.png");
    QIcon line_construct(":/component/construct/line_.png");
    QIcon plane_construct(":/component/construct/plane_.png");
    QIcon circle_construct(":/component/construct/circle_.png");
    QIcon rect_construct(":/component/construct/rectangle_.png");
    QIcon sphere_construct(":/component/construct/sphere_.png");
    QIcon cone_construct(":/component/construct/cone_.png");
    QIcon cylinder_construct(":/component/construct/cylinder_.png");

    constructPoint->setIcon(point_construct);
    constructLine->setIcon(line_construct);
    constructPlane->setIcon(plane_construct);
    constructCircle->setIcon(circle_construct);
    constructRect->setIcon(rect_construct);
    constructSphere->setIcon(sphere_construct);
    constructCylinder->setIcon(cylinder_construct);
    constructCone->setIcon(cone_construct);
    constructDis->setIcon(distanceIcon);
    constructCloud->setIcon(cloudIcon);
    constructAngle->setIcon(angleIcon);
    // 连接到 toolwidget
    connect(constructPoint, &QAction::triggered, this, [&](){ pWinToolWidget->onConstructPoint(); });
    connect(constructLine, &QAction::triggered, this, [&](){ pWinToolWidget->onConstructLine();});
    connect(constructPlane, &QAction::triggered, this, [&](){ pWinToolWidget->onConstructPlane();});
    connect(constructCircle, &QAction::triggered, this, [&](){ pWinToolWidget->onConstructCircle();});
    connect(constructSphere, &QAction::triggered, this, [&](){ pWinToolWidget->onConstructSphere();});
    connect(constructCylinder, &QAction::triggered, this, [&](){ pWinToolWidget->onConstructCylinder();});
    connect(constructCone, &QAction::triggered, this, [&](){ pWinToolWidget->onConstructLine();});
    connect(constructDis, &QAction::triggered, this, [&](){ pWinToolWidget->onConstructDistance();});
    connect(constructCloud, &QAction::triggered, this, [&](){ pWinToolWidget->onConstructPointCloud();});
    connect(constructAngle, &QAction::triggered, this, [&](){ pWinToolWidget->onConstructAngle();});
    connect(constructRect, &QAction::triggered, this, [&](){ pWinToolWidget->onConstructRectangle();});

    QMenu* fittingMenu = bar->addMenu("识别(S)");
    QAction* showFitMenu = new QAction();
    showFitMenu->setShortcut(QKeySequence(Qt::Key_S));
    connect(showFitMenu, &QAction::triggered, this, [this,fittingMenu]() {
        QPoint globalPos = mapToGlobal(QPoint(430, 22));
        fittingMenu->exec(globalPos);
    });
    addAction(showFitMenu);
    QAction* findPoint = fittingMenu->addAction("点");
    QAction* findLine = fittingMenu->addAction("线");
    QAction* findPlane = fittingMenu->addAction("平面");
    QAction* findCirlce = fittingMenu->addAction("圆");
    QAction* findRect = fittingMenu->addAction("矩形");
    QAction* findSphere = fittingMenu->addAction("球体");
    QAction* findCylinder= fittingMenu->addAction("圆柱");
    QAction* findCone = fittingMenu->addAction("圆锥");

    findPoint->setIcon(pointIcon);
    findLine->setIcon(lineIcon);
    findPlane->setIcon(planeIcon);
    findCirlce->setIcon(circleIcon);
    findRect->setIcon(rectangleIcon);
    findSphere->setIcon(sphereIcon);
    findCylinder->setIcon(cylinderIcon);
    findCone->setIcon(coneIcon);

    connect(findPoint, &QAction::triggered, this, [&](){ pWinToolWidget->onFindPoint(); });
    connect(findLine, &QAction::triggered, this, [&](){ pWinToolWidget->onFindLine(); });
    connect(findPlane, &QAction::triggered, this, [&](){ pWinToolWidget->onFindPlane(); });
    connect(findCirlce, &QAction::triggered, this, [&](){ pWinToolWidget->onFindCircle(); });
    connect(findRect, &QAction::triggered, this, [&](){ pWinToolWidget->onFindRectangle(); });
    connect(findSphere, &QAction::triggered, this, [&](){ pWinToolWidget->onFindSphere(); });
    connect(findCylinder, &QAction::triggered, this, [&](){ pWinToolWidget->onFindCylinder(); });
    connect(findCone, &QAction::triggered, this, [&](){ pWinToolWidget->onFindCone(); });

    QMenu *cloudOperation=bar->addMenu("点云操作(V)");
    QAction* showCloudMenu = new QAction();
    showCloudMenu->setShortcut(QKeySequence(Qt::Key_V));
    connect(showCloudMenu, &QAction::triggered, this, [this,cloudOperation]() {
        QPoint globalPos = mapToGlobal(QPoint(490, 22));
        cloudOperation->exec(globalPos);
    });
    addAction(showCloudMenu);
    QAction* compareAction=cloudOperation->addAction("点云对比");
    compareAction->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_C));
    compareAction->setShortcutContext(Qt::ApplicationShortcut);
    connect(compareAction,&QAction::triggered,this,[&](){
        pWinVtkWidget->onCompare();
    });
    QAction* alignAction=cloudOperation->addAction("点云对齐");
    alignAction->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_Q));
    alignAction->setShortcutContext(Qt::ApplicationShortcut);
    connect(alignAction,&QAction::triggered,this,[&](){
        pWinVtkWidget->onAlign();
    });
    QAction* ReconstructionAction=cloudOperation->addAction("点云重建");
    ReconstructionAction->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_Q));
    ReconstructionAction->setShortcutContext(Qt::ApplicationShortcut);
    connect(ReconstructionAction,&QAction::triggered,this,[&](){
        pWinVtkWidget->poissonReconstruction();
    });

    QMenu * switchTheme = bar->addMenu("主题(T)");
    QAction* showThemeMenu = new QAction();
    showThemeMenu->setShortcut(QKeySequence(Qt::Key_T));
    connect(showThemeMenu, &QAction::triggered, this, [this,switchTheme]() {
        QPoint globalPos = mapToGlobal(QPoint(550, 22));
        switchTheme->exec(globalPos);
    });
    addAction(showThemeMenu);
    QAction* lightBlue = switchTheme->addAction("简约蓝(默认)");
    lightBlue->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_D));
    lightBlue->setShortcutContext(Qt::ApplicationShortcut);
    connect(lightBlue, &QAction::triggered, this, &MainWindow::onConvertLighBlueTheme);
    QAction* lightGrey = switchTheme->addAction("浅灰色");
    lightGrey->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_F));
    lightGrey->setShortcutContext(Qt::ApplicationShortcut);
    connect(lightGrey, &QAction::triggered, this, &MainWindow::onConvertLightGreyTheme);
    QAction* darkBlue = switchTheme->addAction("科技蓝");
    darkBlue->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_G));
    darkBlue->setShortcutContext(Qt::ApplicationShortcut);
    connect(darkBlue, &QAction::triggered, this, &MainWindow::onConvertDarkBlueTheme);

    // 添加竖线分隔符
    QFrame *line = new QFrame();
    line->setFrameShape(QFrame::VLine);  // 设置形状为垂直线
    line->setFrameShadow(QFrame::Sunken);  // 设置阴影为凹陷效果，增强3D效果
    line->setLineWidth(2);  // 设置线宽
    line->setStyleSheet("color: black;");  // 设置竖线的颜色为灰色

    //状态栏
    stbar=statusBar();
    setStatusBar(stbar);
    // QLabel *label1=new QLabel("左侧状态栏",this);
    // stbar->addWidget(label1);
    // QLabel *label2=new QLabel("右侧状态栏",this);
    // stbar->addWidget(label2);
    switchRefCsBtn = new QPushButton("依赖坐标系");
    switchRefCsBtn->setStyleSheet("QPushButton { padding: 1px;}");
    switchRefCsBtn->setFixedSize(100, 20);
    switchRefCsBtn->setObjectName("statusSwitchRef");
    switchRefCsBtn->setFlat(true); // 设置按钮为平面样式
    switchRefCsBtn->installEventFilter(this);  // 为按钮安装事件过滤器
    stbar->addWidget(switchRefCsBtn);

    stbar->addWidget(line);

    switchCsBtn = new QPushButton("机械坐标系");
    switchCsBtn->setStyleSheet("QPushButton { padding: 1px;}");
    switchCsBtn->setFixedSize(100, 20);
    switchCsBtn->setFlat(true); // 设置按钮为平面样式
    switchCsBtn->installEventFilter(this);  // 为按钮安装事件过滤器
    switchCsBtn->setObjectName("statusSwitchCs");
    stbar->addWidget(switchCsBtn);

    // stbar->addWidget(line);

    spMainWindow=new QSplitter(Qt::Horizontal,this);
    spLeft=new QSplitter(Qt::Vertical,spMainWindow);
    spMiddleup=new QSplitter(Qt::Vertical,spMainWindow);
    spRightup=new QSplitter(Qt::Vertical,spMainWindow);

    spLeft->addWidget(pWinElementListWidget);

    pWinReportWidget->hide();
    QStackedWidget* centralWidget = new QStackedWidget(this);
    centralWidget->addWidget(pWinVtkWidget);
    centralWidget->addWidget(pWinReportWidget);
    spMiddleup->addWidget(centralWidget);
    // 连接切换窗口菜单
    connect(showVtkWin, &QAction::triggered, this, [centralWidget]() {
        centralWidget->setCurrentIndex(0); // 显示主窗口
    });
    connect(showReportWin, &QAction::triggered, this, [centralWidget]() {
        centralWidget->setCurrentIndex(1); // 显示报表窗口
    });

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
    spMainWindow->setContentsMargins(0, 0, 0, 0);

    spMainWindow->setHandleWidth(5);

    setCentralWidget(spMainWindow);
    ContralWidget * contralWidget=new ContralWidget(pWinToolWidget,this);
    connect(contralAction,&QAction::triggered,[=](){contralWidget->show();});
    connect(actorAdjust, &QAction::triggered, [&](){pWinVtkWidget->createActorController();});

}

void MainWindow::LoadWidgets(){
    pWinVtkPresetWidget=new VtkPresetWidget(this);
    pWinDataWidget=new DataWidget(this);
    pWinElementListWidget=new ElementListWidget(this);
    pWinFileManagerWidget=new FileManagerWidget(this);
    pWinToolWidget=new ToolWidget(this);
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
    QStringList filePaths;
    if(peopleOpenfile){
        // 使用QFileDialog打开文件对话框，允许多选
        filePaths = QFileDialog::getOpenFileNames(this, tr("Open Files"));
        //QString filePath = QFileDialog::getOpenFileName(this, tr("open  file"));
        if (filePaths.isEmpty()) { // 如果没有选择文件，返回
            return;
        }
    }else{
        filePaths.append(filePathChange);
    }
    for (const QString &filePath : filePaths) {
        QFileInfo fileInfo(filePath);
        QString fileName = fileInfo.fileName();
        auto fileLowName = fileName.toLower();
        // 根据文件名是否含有 "stand"来判断，先转成小写
        if (fileLowName.contains("stand")) {
            modelCloudExist=true; // 用于保证后续序列化文件

            //在modelFileMap中添加添加新文件，并分配新的cloud
            getpWinFileMgr()->getModelFileMap().insert(filePath, true);
            auto cloud = getPointCloudListMgr()->CreateCloudFromFile(filePath);
            cloud->isModelCloud=true;
            cloud->m_strAutoName += "(标准)";
            getPWinToolWidget()->addToList(cloud);

            // 给拟合的临时点云指针赋值
            auto newcloud = new pcl::PointCloud<pcl::PointXYZRGB>(cloud->m_pointCloud);

            auto tmpCloud=getPointCloudListMgr()->getTempCloud();
            pcl::copyPointCloud(*newcloud, tmpCloud);

            getpWinFileMgr()->cloudptr = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(newcloud);
            if(!getpWinFileMgr()->cloudptr){
                qDebug() << "拟合用的点云为空!";
            }
            NotifySubscribe();
            getPWinVtkWidget()->onTopView();

            //pWinFileManagerWidget->openModelFile(fileName, filePath);
            pWinVtkPresetWidget->setWidget(fileName+"文件已打开");
        } else{
            //在measuredFileMap中添加新文件，并分配新的cloud
            getpWinFileMgr()->getMeasuredFileMap().insert(filePath, true);
            auto cloud = getPointCloudListMgr()->CreateCloudFromFile(filePath);
            cloud->isMeasureCloud=true;
            cloud->m_strAutoName += "(实测)";
            getPWinToolWidget()->addToList(cloud);

            // 给拟合的临时点云指针赋值
            auto newcloud = new pcl::PointCloud<pcl::PointXYZRGB>(cloud->m_pointCloud);
            getpWinFileMgr()->cloudptr = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(newcloud);
            NotifySubscribe();
            getPWinVtkWidget()->onTopView();
            getPWinElementListWidget()->onAddElement(getpWinFileMgr()->cloudptr);

            //pWinFileManagerWidget->openMeasuredFile(fileName, filePath);
            pWinVtkPresetWidget->setWidget(fileName+"文件已打开");
            //pWinElementListWidget->onAddElement();
        }
        if(filePath.endsWith("qins")){
            QFile file(filePath);
            if (!file.open(QIODevice::ReadOnly)) {
                // 如果文件无法打开，输出错误信息
                qDebug() << "Failed to open file:" << file.errorString();
                return;
            }

            //deserialize
            QDataStream in(&file);
            in.setVersion(QDataStream::Qt_6_0);

            in>>*m_EntityListMgr;
            in>>*m_ObjectListMgr;
            pWinSetDataWidget->deserialize(in);
            in>>pWinFileMgr->getContentItemMap();
            // in>>pWinFileMgr->getIdentifyItemMap();
            in>>pWinFileMgr->getModelFileMap();

            // //去除原来构建的点云
            // pWinFileMgr->removePointCloudKeys(pWinFileMgr->getContentItemMap());

            //打开模型点云
            in>>modelCloudExist;
            if(modelCloudExist){
                CEntity* entity=new CPointCloud();
                entity->deserialize(in);
                m_EntityListMgr->m_entityList.append(entity);
                m_ObjectListMgr->getObjectList().append(entity);
            }

            //反序列化toolWidget中的list
            pWinToolWidget->deserializeEntityList(in,pWinToolWidget->getConstructEntityList()); //构造
            //pWinToolWidget->deserializeEntityList(in,pWinToolWidget->getIdentifyEntityList()); //拟合

            qDebug() << "加载成功,ConstructEntityList的大小为:"<<pWinToolWidget->getConstructEntityList().size();
            qDebug() << "加载成功,IdentifyEntityList的大小为:"<<pWinToolWidget->getIdentifyEntityList().size();

            qDebug() << "加载成功,m_EntityListMgr的大小为:"<<m_EntityListMgr->getEntityList().size()<<"m_ObjectListMgr的大小为:"<<m_ObjectListMgr->getObjectList().size();
            qDebug()<<"首个Object的类型为:"<<m_ObjectListMgr->GetAt(0)->GetUniqueType();
            NotifySubscribe();
            pWinVtkPresetWidget->setWidget(fileName+"文件已打开");
            file.close();
        }
    }

    // if (filePath.endsWith("stp")) {
    //     QFileInfo fileInfo(filePath);
    //     QString fileName = fileInfo.fileName();
    //     pWinFileManagerWidget->openModelFile(fileName,filePath);
    // }else if(filePath.endsWith("stl")||filePath.endsWith("pcd")||filePath.endsWith("ply")){
    //     QFileInfo fileInfo(filePath);
    //     QString fileName = fileInfo.fileName();
    //     pWinFileManagerWidget->openMeasuredFile(fileName,filePath);
    //     }
}
void MainWindow::saveFile(){
    //保存ObjectList
    // QString filePath = QFileDialog::getSaveFileName(nullptr, QString("Save As"), "请输入文件名", QString("dat(*.dat )"));
    QString filePath = QFileDialog::getSaveFileName(nullptr, QString("Save As"), "请输入文件名", QString("qins(*.qins);;所有文件 (*)"));

    if (filePath.isEmpty()){
        return ;
    }

    if (QFileInfo(filePath).suffix().isEmpty()){
        // filePath.append(".dat");
        filePath.append(".qins");
    }

    // 创建 QFile 对象
    QFile file(filePath);

    // 打开文件以进行写入
    if (!file.open(QIODevice::WriteOnly)) {
        qWarning() << "无法打开文件:" << file.errorString();
        return;
    }

    //serialize
    QDataStream out(&file);
    out.setVersion(QDataStream::Qt_6_0);

    // 写入内容
    // out<<*(getObjectListMgr()); // 返回为指针，需要解引用
    if(m_EntityListMgr){
        out<<*m_EntityListMgr;
        out<<*m_ObjectListMgr;
        pWinSetDataWidget->serialize(out);
        out<<pWinFileMgr->getContentItemMap();
        // out<<pWinFileMgr->getIdentifyItemMap();
        out<<pWinFileMgr->getModelFileMap();

        //保存模型点云
        out<<modelCloudExist;
        for(int i=0;i<getEntityListMgr()->getEntityList().size();i++){
            if(getEntityListMgr()->getEntityList()[i]->GetUniqueType()==enPointCloud){
                CPointCloud*could=(CPointCloud*)getEntityListMgr()->getEntityList()[i];
                if(could->isModelCloud){
                    could->serialize(out);
                    break;
                }
            }
        }

        //序列化toolWidget中的list
        pWinToolWidget->serializeEntityList(out,pWinToolWidget->getConstructEntityList()); //构造
        //pWinToolWidget->serializeEntityList(out,pWinToolWidget->getIdentifyEntityList()); //拟合

    }else{
        qWarning("Entity manager is null, nothing to save.");
    }


    // 关闭文件
    file.close();
    QMessageBox::information(nullptr, "提示", "qins文件保存成功");
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

void MainWindow::loadManager()
{
    m_pcsListMgr=new CPcsMgr();
    m_ObjectListMgr=new CObjectMgr();
    m_EntityListMgr=new CEntityMgr();
    m_ChosenListMgr=new ChosenCEntityMgr();

    pWinFileMgr=new FileMgr();
    m_CloudListMgr = new PointCloudListMgr();

    m_pcsListMgr->Initialize();
}

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
    case enCuboid:
        pTempEntity = new CCuboid();
        pTempEntity->setEntityType(enCuboid);
        break;
    case enSurfaces:
        pTempEntity = new CSurfaces();
        pTempEntity->setEntityType(enSurfaces);
        break;
    default:
        break;
    }
    pTempEntity->setDwAddress((uintptr_t)(pTempEntity));
    return pTempEntity;
}

void MainWindow::NotifySubscribe()
{
    pWinElementListWidget->upadteelementlist();
    pWinFileManagerWidget->UpdateInfo();
    pWinVtkWidget->UpdateInfo(); // 更新vtkwidget信息

}

void MainWindow::OnPresetPoint(CPosition pt){

    //qDebug()<<"clicked3.1";
    CPoint *pPoint = (CPoint *)CreateEntity(enPoint);
    pPoint->Form="预制";
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
    NotifySubscribe();
}

void MainWindow::OnPresetLine(CPosition ptStart, CPosition ptEnd)
{
    CLine *pLine = (CLine *)CreateEntity(enLine);
    pLine->Form="预制";
    pLine->setBegin(ptStart);
    pLine->setEnd(ptEnd);
    pLine->setCurrentId();
    pLine->m_CreateForm = ePreset;
    pLine->m_pRefCoord = m_pcsListMgr->m_pPcsCurrent;
    pLine->m_pCurCoord = m_pcsListMgr->m_pPcsCurrent;
    pLine->m_pExtCoord = m_pcsListMgr->m_pPcsCurrent;
    // pLine->SetNominal();
    // 加入Entitylist 和 ObjectList
    m_EntityListMgr->Add(pLine);
    m_ObjectListMgr->Add(pLine);

    NotifySubscribe();
}

void MainWindow::OnPresetCircle(CPosition pt, double diameter,QVector4D normal)
{
    //CPosition pt(1,1,1);
    //double diameter = 1.5;

    CCircle *pCircle = (CCircle *)CreateEntity(enCircle);
    pCircle->setCurrentId();
    pCircle->Form="预制";
    pCircle->SetCenter(pt);
    pCircle->SetDiameter(diameter);
    pCircle->setNormal(normal);
    pCircle->m_CreateForm = ePreset;
    pCircle->m_pRefCoord = m_pcsListMgr->m_pPcsCurrent;
    pCircle->m_pCurCoord = m_pcsListMgr->m_pPcsCurrent;
    pCircle->m_pExtCoord = m_pcsListMgr->m_pPcsCurrent;
    // pCircle->SetNominal();

    // std::vector<QPointF> points;

    //pCircle->addToolArray((CircleGraphics*)CreateImgTool(eImgCircleGraphics,points,data));

    // 加入Entitylist 和 ObjectList
    m_EntityListMgr->Add(pCircle);
    m_ObjectListMgr->Add(pCircle);

    // if(m_ObjectListMgr->GetInsertPos()==-1){
    //     m_ObjectListMgr->Add(pCircle);
    //     pWinElementListWidget->updateInsertIndicatorPosition();

    // }else{
    //     m_ObjectListMgr->Insert(pCircle, m_ObjectListMgr->m_nInsertPos);
    //     m_ObjectListMgr->m_nInsertPos++;
    //     pWinElementListWidget->updateInsertIndicatorPosition();
    // }


    //选中元素,取消其他元素选中
    for(auto const &object:m_ObjectListMgr->getObjectList()){
        object->SetSelected(false);
    }
    pCircle->SetSelected(true);

    NotifySubscribe();
}

void MainWindow::OnPresetPlane(CPosition posCenter, QVector4D normal, QVector4D direction, double length, double width)
{
    CPlane *pPlane = (CPlane *)CreateEntity(enPlane);
    pPlane->setCurrentId();
    pPlane->Form="预制";
    pPlane->setCenter(posCenter);
    pPlane->setDir_long_edge(direction);
    pPlane->setNormal(normal);
    pPlane->setLength(length);
    pPlane->setWidth(width);

    pPlane->m_CreateForm = ePreset;
    pPlane->SetCurCoord(m_pcsListMgr->m_pPcsCurrent);
    pPlane->SetRefCoord(m_pcsListMgr->m_pPcsCurrent);
    pPlane->SetExtCoord(m_pcsListMgr->m_pPcsCurrent);
    // pPlane->SetNominal();

    // 加入Entitylist 和 ObjectList
    m_EntityListMgr->Add(pPlane);
    m_ObjectListMgr->Add(pPlane);

    NotifySubscribe();

}

void MainWindow::OnPresetSphere(CPosition posCenter, double diametre)
{
    CSphere *pSphere = (CSphere *)CreateEntity(enSphere);
    pSphere->setCurrentId();
    pSphere->Form="预制";
    pSphere->setCenter(posCenter);
    pSphere->setDiameter(diametre);

    pSphere->m_CreateForm = ePreset;
    pSphere->SetCurCoord(m_pcsListMgr->m_pPcsCurrent);
    pSphere->SetRefCoord(m_pcsListMgr->m_pPcsCurrent);
    pSphere->SetExtCoord(m_pcsListMgr->m_pPcsCurrent);
    // pSphere->SetNominal();

    // 加入Entitylist 和 ObjectList
    m_EntityListMgr->Add(pSphere);
    m_ObjectListMgr->Add(pSphere);
    qDebug()<<"add a sphere";
    NotifySubscribe();
}

void MainWindow::OnPresetCylinder(CPosition pos, QVector4D vec, double height, double diametre)
{
    CCylinder *pCylinder = (CCylinder *)CreateEntity(enCylinder);
    pCylinder->setCurrentId();
    pCylinder->Form="预制";
    pCylinder->setAxis(vec);
    pCylinder->setBtm_center(pos);
    pCylinder->setDiameter(diametre);
    pCylinder->setHeight(height);

    pCylinder->m_CreateForm = ePreset;
    pCylinder->SetCurCoord(m_pcsListMgr->m_pPcsCurrent);
    pCylinder->SetRefCoord(m_pcsListMgr->m_pPcsCurrent);
    pCylinder->SetExtCoord(m_pcsListMgr->m_pPcsCurrent);
    // pCylinder->SetNominal();

    // 加入Entitylist 和 ObjectList
    m_EntityListMgr->Add(pCylinder);
    m_ObjectListMgr->Add(pCylinder);

    qDebug()<<"add a Cone";
    NotifySubscribe();

}


void MainWindow::OnPresetCone(CPosition posCenter, QVector4D axis, double partH, double fullH, double angle)

{
    CCone *pCone = (CCone *)CreateEntity(enCone);
    pCone->setCurrentId();
    pCone->Form="预制";
    pCone->setCone_height(partH);
    pCone->setHeight(fullH);
    pCone->setAxis(axis);
    pCone->setRadian(angle);
    pCone->setVertex(posCenter);

    pCone->m_CreateForm = ePreset;
    pCone->SetCurCoord(m_pcsListMgr->m_pPcsCurrent);
    pCone->SetRefCoord(m_pcsListMgr->m_pPcsCurrent);
    pCone->SetExtCoord(m_pcsListMgr->m_pPcsCurrent);
    // pCone->SetNominal();

    // 加入Entitylist 和 ObjectList
    m_EntityListMgr->Add(pCone);
    m_ObjectListMgr->Add(pCone);

    qDebug()<<"add a Cone";
    NotifySubscribe();
}

void MainWindow::OnPresetCuboid(CPosition posCenter,double length,double width,double height,QVector4D normal)
{
    CCuboid *pCuboid=(CCuboid *)CreateEntity(enCuboid);
    pCuboid->setCurrentId();
    pCuboid->Form="预制";
    pCuboid->setCenter(posCenter);;
    pCuboid->setLength(length);
    pCuboid->setWidth(width);
    pCuboid->setHeight(height);
    pCuboid->setNormal(normal);

    pCuboid->m_CreateForm = ePreset;
    pCuboid->SetCurCoord(m_pcsListMgr->m_pPcsCurrent);
    pCuboid->SetRefCoord(m_pcsListMgr->m_pPcsCurrent);
    pCuboid->SetExtCoord(m_pcsListMgr->m_pPcsCurrent);

    m_EntityListMgr->Add(pCuboid);
    m_ObjectListMgr->Add(pCuboid);

    NotifySubscribe();
}

CObjectMgr *MainWindow::getObjectListMgr()
{
    return m_ObjectListMgr;
}

CEntityMgr *MainWindow::getEntityListMgr()
{
    return m_EntityListMgr;
}

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
    case 7:pWinPresetElemWidget->tabWidget->setCurrentIndex(7);
        break;
    }
    pWinPresetElemWidget->show();
}

bool MainWindow::eventFilter(QObject* watched, QEvent* event){
    if (event->type() == QEvent::MouseButtonDblClick && qobject_cast<QPushButton*>(watched)) {
        // 检测到双击事件
        QPushButton* button = qobject_cast<QPushButton*>(watched);
        if(button->objectName()=="statusSwitchCs")
        {
            // QMessageBox::information(this, "双击事件", "切换坐标系双击了!");
            QDialog dialog(this);
            dialog.setWindowTitle("请选择坐标系");
            QVBoxLayout layout(&dialog);
            QListWidget listWidget; //dialog中的list
            layout.addWidget(&listWidget);
            QString strClickedText; //记录点击list的item

            if(m_pcsListMgr->m_bTempPcsNodeInUse)
            {
                listWidget.addItem("临时坐标系");
            }

            // 使用反向迭代器倒着遍历
            for (auto it = m_pcsListMgr->m_PcsNodeList.rbegin(); it != m_pcsListMgr->m_PcsNodeList.rend(); ++it) {
                CPcsNode* node = *it;  // 解引用迭代器以获取 CPcsNode* 指针
                if(node->m_bDeleted==false)
                    listWidget.addItem(node->GetObjectCName());
            }

            // 连接列表的点击事件
            connect(&listWidget, &QListWidget::itemClicked, this,[&](QListWidgetItem *item){
                // QMessageBox::information(this, "选项被点击", "你点击了: " + item->text());
                strClickedText=item->text();
            });

            // 添加按钮
            QHBoxLayout *buttonLayout = new QHBoxLayout;
            QPushButton *okButton = new QPushButton("确定");
            QPushButton *cancelButton = new QPushButton("取消");
            buttonLayout->addWidget(okButton);
            buttonLayout->addWidget(cancelButton);
            layout.addLayout(buttonLayout);

            // 连接按钮信号
            connect(okButton, &QPushButton::clicked, &dialog, &QDialog::accept);
            connect(cancelButton, &QPushButton::clicked, &dialog, &QDialog::reject);

            // 显示对话框
            if (dialog.exec() == QDialog::Accepted) {
                if(!strClickedText.isEmpty())
                {
                    if(strClickedText=="临时坐标系")
                    {
                        // 什么也不做
                    }
                    else if (strClickedText=="机械坐标系")
                    {
                        CPcs* pPcs = m_pcsListMgr->GetBaseCoordSystem();
                        m_pcsListMgr->SetCurCoordSystem(pPcs);
                        NotifySubscribe();
                        // 更新所有实体的当前坐标系
                        for(CEntity *pEntity : m_EntityListMgr->getEntityList())
                        {
                            pEntity->SetCurCoord(pPcs);
                        }
                    }
                    else
                    {
                        // 遍历找到该工件坐标系
                        CPcs* pPcs = m_pcsListMgr->Find(strClickedText);
                        m_pcsListMgr->SetCurCoordSystem(pPcs);
                        NotifySubscribe();
                        for(CEntity *pEntity : m_EntityListMgr->getEntityList())
                        {
                            pEntity->SetCurCoord(pPcs);
                        }
                    }
                    // 右下加标签设置为所选坐标系
                    button->setText(strClickedText);
                }
                else
                {
                    stbar->showMessage("您没有选择任何坐标系",2000);
                }
            }
            dialog.close();

        }
        else if(button->objectName()=="statusSwitchRef")
        {
            // 获取选中元素的下表index
            int index=-1;
            for(int i=0;i<m_EntityListMgr->getEntityList().size();i++)
            {
                if(m_EntityListMgr->getEntityList()[i]->IsSelected() == true) // 如果对象被选中
                {
                    index=i;
                }
            }

            if(button->text()=="参考依赖坐标系")
            {
                button->setText("参考当前坐标系");
                m_nRelyOnWhichCs = csCur;
                pWinDataWidget->updateinfo(); // 更新数据结果窗口
            }
            else
            {
                button->setText("参考依赖坐标系");
                m_nRelyOnWhichCs = csRef;
                pWinDataWidget->updateinfo(); // 更新数据结果窗口
            }
        }

        return true;  // 返回 true 表示事件已被处理，不再传递
    }
    // 调用基类的事件过滤器以处理其他事件
    return QMainWindow::eventFilter(watched, event);
}

//处理临时坐标系的创建(只有点和圆可建立坐标系)
void MainWindow::on2dCoordOriginAuto(){
    // 如果临时坐标系不为空，则禁止继续创建临时坐标系
    if(m_pcsListMgr->m_bTempPcsNodeInUse)
    {
        stbar->showMessage("已存在临时坐标系",2000);
        qDebug()<<"临时坐标系不为空";
        return;
    }
    //qDebug()<<"添加坐标系";

    m_pcsListMgr->m_bTempPcsNodeInUse = true;

    // 创建一个对象列表，用于存储选中的对象
    QVector<CObject*> choosenList;
    int index=-1;
    for(int i=0;i<m_ObjectListMgr->getObjectList().size();i++)
    {
        if(m_ObjectListMgr->getObjectList()[i]->IsSelected() == true) // 如果对象被选中
        {
            choosenList.push_back(m_ObjectListMgr->getObjectList()[i]);
            index=i;
            qDebug()<<"列表里有东西";
        }
    }

    if(choosenList.size() != 1) {
        qDebug()<<"size不为1";
        return;
    }
    CPosition pos;

    // 根据选中的对象类型，获取其坐标信息
    switch (choosenList[0]->GetUniqueType()) {
    case enCircle:
    {
        CCircle* newCircle = (CCircle*)choosenList[0];
        pos.x=newCircle->getCenter().x; //当前坐标系下预置时填写的坐标
        pos.y=newCircle->getCenter().y;
        pos.z=newCircle->getCenter().z;
        break;
    }
    case enPoint:
    {
        CPoint* newPoint = (CPoint*)choosenList[0];
        pos.x=newPoint->GetPt().x;
        pos.y=newPoint->GetPt().y;
        pos.z=newPoint->GetPt().z;
        break;
    }
    default:
        break;
    }
    CPcs *pPcs = new CPcs();
    QVector4D posVec = m_pcsListMgr->m_pPcsCurrent->m_mat * QVector4D(pos.x, pos.y, pos.z, 1); // 得到全局坐标
    CPosition globalPos(posVec.x(), posVec.y(), posVec.z());

    // 设置新坐标系的矩阵为当前坐标系的矩阵
    pPcs->m_mat = m_pcsListMgr->m_pPcsCurrent->m_mat;
    // 将坐标系平移到选中对象的位置，即工件坐标系
    pPcs->m_mat.translate(pos.x, pos.y, pos.z);

    // 上面是局部变量，但是坐标系中心必须保存全局坐标
    pPcs->m_poso = globalPos;

    pPcs->m_nPcsID = m_pcsListMgr->m_pPcsCurrent->m_nPcsID;
    pPcs->m_nRef = m_pcsListMgr->m_nCount+1;

    // 创建节点
    CPcsNode *pcsTempNode = new CPcsNode();
    pcsTempNode->pPcs = pPcs;
    // pcsTempNode->create_type = e2dCoord; // 记录坐标系如何创建
    pcsTempNode->SetObjectAutoName("临时坐标系");
    pcsTempNode->SetObjectCName("临时坐标系");

    m_pcsListMgr->m_pNodeTemporary = pcsTempNode;

    // 将临时坐标系设置为当前坐标系,并更新所有entity的当前坐标系
    m_pcsListMgr->SetCurCoordSystem(pcsTempNode->pPcs);
    for (CEntity* pEntity : m_EntityListMgr->getEntityList()) {
        pEntity->SetCurCoord(pPcs);
    }
    //qDebug()<<"执行到这步了";
    m_pcsListMgr->m_pNodeTemporary->setDwAddress((uintptr_t)(pcsTempNode)); // 将节点地址转换为uintptr_t类型并存储
    m_ObjectListMgr->Add(m_pcsListMgr->m_pNodeTemporary);
    //选中元素,取消其他元素选中
    for(auto const &object:m_ObjectListMgr->getObjectList()){
        object->SetSelected(false);
    }
    m_pcsListMgr->m_pNodeTemporary->SetSelected(true); // 将临时坐标系节点设置为选中状态
    // 更新状态栏
    switchCsBtn->setText("临时坐标系");

    //qDebug()<<"添加坐标系完成！！";
    NotifySubscribe();
    m_pcsListMgr->m_bTempPcsNodeInUse = true;
}

// 将临时坐标系转化为正式的坐标系并保存
void MainWindow::on2dCoordSave(){
    if(m_pcsListMgr->bTempPcsNodeInUse() == false)
    {
        stbar->showMessage("没有临时坐标系需要保存",2000);
        return;
    }

    // 临时坐标系转为正式坐标系
    m_pcsListMgr->AddCoordSys(m_pcsListMgr->m_pNodeTemporary);
    m_pcsListMgr->m_pTailPcsNode->nPcsNodeId = ++CPcsNode::nPcsNodeCount;
    int nPcsId=m_pcsListMgr->m_pTailPcsNode->nPcsNodeId;
    QString pcsName=QString("工件坐标系%1").arg(nPcsId-1); // 命名为 "工件坐标系" + (ID - 1)
    m_pcsListMgr->m_pTailPcsNode->SetObjectAutoName(pcsName);
    m_pcsListMgr->m_pTailPcsNode->SetObjectCName(pcsName);
    qDebug()<<"当前有"<<m_pcsListMgr->m_nCount<<"个坐标系";

    m_pcsListMgr->setBTempPcsNodeInUse(false);  // 将标志设为不再使用临时坐标系

    //更新状态栏
    switchCsBtn->setText(pcsName);
    NotifySubscribe();
}

ElementListWidget* MainWindow:: getPWinElementListWidget(){
    return pWinElementListWidget;
}
VtkWidget *MainWindow:: getPWinVtkWidget(){
    return pWinVtkWidget;
}

DataWidget *MainWindow::getPWinDataWidget()
{
    return pWinDataWidget;
}

ToolWidget *MainWindow::getPWinToolWidget()
{
    return pWinToolWidget;
}



void MainWindow::onTopViewClicked()
{
    pWinVtkWidget->onTopView();
}

void MainWindow::onRightViewClicked()
{
    pWinVtkWidget->onRightView();
}

void MainWindow::onFrontViewClicked()
{
    pWinVtkWidget->onFrontView();
}

void MainWindow::onIsometricViewClicked()
{
    pWinVtkWidget->onIsometricView();
}

void MainWindow::filechange()
{
    // fileWatcher.addPath("C:/Users/Lenovo/Desktop/downFTPfile"); // 替换为FTP目录路径
    // connect(&fileWatcher, &QFileSystemWatcher::directoryChanged, this, &MainWindow::onFileChanged);
    // // 初始化文件处理定时器
    // fileProcessorTimer.setInterval(1000); // 每秒处理一个文件
    // connect(&fileProcessorTimer, &QTimer::timeout, this, &MainWindow::processNextFile);
    // // 初始化已存在的文件列表
    // QDir dir("C:/Users/Lenovo/Desktop/downFTPfile");
    // existingFiles = dir.entryList(QDir::Files);
}

void MainWindow::onFileChanged(const QString &path)
{
    qDebug() << "文件发生变化：" << path;
    // 获取目录中的所有文件
    QDir dir(path);
    QStringList currentFiles = dir.entryList(QDir::Files);
    // 检查新增的文件
    foreach (const QString &file, currentFiles) {
        if (!existingFiles.contains(file)) {
            QString filePath = path + "/" + file;
            qDebug() << "发现新文件：" << filePath;

            // 根据文件扩展名判断优先级
            if (file.endsWith(".ply", Qt::CaseInsensitive)) {
                // 如果是.ply文件，优先添加到队列前面
                fileQueue.prepend(filePath);
                qDebug() << "优先添加.ply文件到队列：" << filePath;
            } else {
                // 其他文件正常添加到队列末尾
                fileQueue.enqueue(filePath);
                qDebug() << "添加到队列：" << filePath;
            }
            // 更新已存在的文件列表
            existingFiles.append(file);
        }
    }
    peopleOpenfile=false;
    //processNextFile();
    // 如果定时器未启动，启动定时器
    if (!fileProcessorTimer.isActive()) {
        fileProcessorTimer.start();
    }
}

void MainWindow::processNextFile()
{
    if (fileQueue.isEmpty()) {
        peopleOpenfile=true;
        fileProcessorTimer.stop(); // 如果队列为空，停止定时器
        return;
    }

    filePathChange = fileQueue.dequeue();
    qDebug() << "正在处理文件：" << filePathChange;
    openFile();
}

double MainWindow::AxesRotateX()
{
    double angleInDegrees = -1;
    for(int i=0;i<m_ObjectListMgr->getObjectList().size();i++)
    {
        if(m_ObjectListMgr->getObjectList()[i]->IsSelected() == true&&
            m_ObjectListMgr->getObjectList()[i]->GetUniqueType() == enLine)
        {
            CLine* newLine = (CLine*)m_ObjectListMgr->getObjectList()[i];
            // 计算向量差
            double deltaX = newLine->getPosition2().x - newLine->getPosition1().x;
            double deltaY = newLine->getPosition2().y - newLine->getPosition1().y;

            // 使用atan2计算旋转角度 (弧度制)
            double angle = atan2(deltaY, deltaX);

            // 将弧度转换为角度
            angleInDegrees = vtkMath::DegreesFromRadians(angle);
            qDebug()<<"angleInDegrees is "<<angleInDegrees;
            break;
        }
    }
    if (angleInDegrees == -1) {
        return -1;  // 没有找到有效的线对象
    }
    return angleInDegrees;
    //pWinVtkWidget->rotateCameraClockwiseInXOY(angleInDegrees);
}

void MainWindow::on2dCoordSetRightX()
{
    double angle = AxesRotateX(); // 获取摆正角度
    if(angle == -1) return;
    m_pcsListMgr->m_pNodeTemporary->pPcs->PlanarRotateXinXY(angle);
    NotifySubscribe();
}

void MainWindow::on2dCoordSetRightY()
{
    double angleX=AxesRotateX();
    if(angleX == -1) return;
    double angle=0; // 线段与y轴正方向夹角
    if(angleX>0){
        angle=fabs(90-angleX);
    }else{
        if(angleX>-90)
            angle=90+fabs(angleX);
        else
            angle=270-fabs(angleX);
    }
    m_pcsListMgr->m_pNodeTemporary->pPcs->PlanarRotateYinXY(angle);
    NotifySubscribe();
}

void MainWindow::onConvertLightGreyTheme()
{
    auto styleFile = QFile(":/style/light_grey.qss");
    styleFile.open(QFile::ReadOnly);
    if(styleFile.isOpen()){
        QString styleSheets = QLatin1String(styleFile.readAll());
        this->setStyleSheet(styleSheets);
        styleFile.close();
    }
    else {
        qDebug() << "打开样式文件失败";
    }
    // 调整渲染颜色以适应主题
    MainWindow::ActorColor[0] = 0.5;
    MainWindow::ActorColor[1] = 0.5;
    MainWindow::ActorColor[2] = 0.5;
    MainWindow::HighLightColor[0] = 1;
    MainWindow::HighLightColor[1] = 1;
    MainWindow::HighLightColor[2] = 0;
    MainWindow::InfoTextColor[0] = 0;
    MainWindow::InfoTextColor[1] = 0;
    MainWindow::InfoTextColor[2] = 0;

    pWinVtkWidget->getRenderer()->SetBackground(0.5, 0.5, 0.5);
    pWinVtkWidget->getRenderer()->SetBackground(1, 1, 1);
    pWinVtkWidget->getRenderer()->SetGradientBackground(true);
    pWinVtkWidget->UpdateInfo();
}

void MainWindow::onConvertLighBlueTheme()
{
    auto styleFile = QFile(":/style/light_blue.qss");
    styleFile.open(QFile::ReadOnly);
    if(styleFile.isOpen()){
        QString styleSheets = QLatin1String(styleFile.readAll());
        this->setStyleSheet(styleSheets);
        styleFile.close();
    }
    else {
        qDebug() << "打开样式文件失败";
    }
    // 调整渲染颜色以适应主题
    MainWindow::ActorColor[0] = 1;
    MainWindow::ActorColor[1] = 1;
    MainWindow::ActorColor[2] = 1;
    MainWindow::HighLightColor[0] = 1;
    MainWindow::HighLightColor[1] = 0;
    MainWindow::HighLightColor[2] = 0;
    MainWindow::InfoTextColor[0] = 1;
    MainWindow::InfoTextColor[1] = 1;
    MainWindow::InfoTextColor[2] = 1;

    pWinVtkWidget->getRenderer()->SetBackground(0.2, 0.3, 0.5);
    pWinVtkWidget->getRenderer()->SetGradientBackground(true);
    pWinVtkWidget->UpdateInfo();
}

void MainWindow::onConvertDarkBlueTheme()
{
    auto styleFile = QFile(":/style/dark_blue.qss");
    styleFile.open(QFile::ReadOnly);
    if(styleFile.isOpen()){
        QString styleSheets = QLatin1String(styleFile.readAll());
        this->setStyleSheet(styleSheets);
        styleFile.close();
    }
    else {
        qDebug() << "打开样式文件失败";
    }
    // 调整渲染颜色以适应主题
    MainWindow::ActorColor[0] = 1;
    MainWindow::ActorColor[1] = 1;
    MainWindow::ActorColor[2] = 1;
    MainWindow::HighLightColor[0] = 1;
    MainWindow::HighLightColor[1] = 0;
    MainWindow::HighLightColor[2] = 0;
    MainWindow::InfoTextColor[0] = 1;
    MainWindow::InfoTextColor[1] = 1;

    pWinVtkWidget->getRenderer()->SetBackground(0.1, 0.2, 0.3);
    pWinVtkWidget->getRenderer()->SetGradientBackground(true);
    pWinVtkWidget->UpdateInfo();
}

FileMgr *MainWindow::getpWinFileMgr(){
    return pWinFileMgr;
}

ChosenCEntityMgr *MainWindow::getChosenListMgr()
{
    return m_ChosenListMgr;
}

PointCloudListMgr *MainWindow::getPointCloudListMgr()
{
    return m_CloudListMgr;
}

void MainWindow::LoadSetDataWidget(){
    pWinSetDataWidget=new setDataWidget();
}

void MainWindow::SetUpTheme()
{
    // auto styleFile = QFile("E:\\QSS样式表大合集\\others\\11.qss");
    auto styleFile = QFile(":/style/light_blue.qss");
    styleFile.open(QFile::ReadOnly);
    if(styleFile.isOpen()){
        QString styleSheets = QLatin1String(styleFile.readAll());
        this->setStyleSheet(styleSheets);
        styleFile.close();
    }
    else {
        qDebug() << "打开样式文件失败";
    }
    // 调整渲染颜色以适应主题
    MainWindow::ActorColor[0] = 1;
    MainWindow::ActorColor[1] = 1;
    MainWindow::ActorColor[2] = 1;
    MainWindow::HighLightColor[0] = 1;
    MainWindow::HighLightColor[1] = 0;
    MainWindow::HighLightColor[2] = 0;

    pWinVtkWidget->getRenderer()->SetBackground(0.1, 0.2, 0.4);
    NotifySubscribe();
}

void MainWindow::SaveIniFile()
{
    QSettings settings("E:/config.ini", QSettings::IniFormat);
    settings.beginGroup("CPoint");
    settings.setValue("x", 2);
    settings.setValue("y", 2);
    settings.setValue("z", 1);
    settings.endGroup();
    settings.sync();
    double x;
    settings.beginGroup("CPoint");
    x = settings.value("x").toDouble();
    settings.endGroup();
    qDebug() << x;

}

setDataWidget *MainWindow::getPWinSetDataWidget(){
    return pWinSetDataWidget;
}
QMap<vtkSmartPointer<vtkActor>, CEntity*>& MainWindow::getactorToEntityMap(){
    return  actorToEntityMap;
}

void MainWindow::Createruler()
{
    getPWinVtkWidget()->createScaleBar();
    getPWinVtkWidget()->attachInteractor();
}

VtkPresetWidget *MainWindow::getPWinVtkPresetWidget(){
    return pWinVtkPresetWidget;
}
