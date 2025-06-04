#include "elementlistwidget.h"
#include "mainwindow.h"
#include "manager/filemgr.h"
#include "vtkwindow/vtkpresetwidget.h"
#include <QDebug>
#include <functional>
int ElementListWidget::pcscount = 0;
ElementListWidget::ElementListWidget(QWidget *parent)
    : QWidget{parent}
{
    m_pMainWin=(MainWindow*)parent;
    auto *layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0); // 去除布局的边距
    layout->setSpacing(0); // 去除布局内部的间距

    // 删除选中元素的按钮
    //QPushButton *deleteButton = new QPushButton("删除选中元素", this);
    //(deleteButton, &QPushButton::clicked, this, &ElementListWidget::onDeleteEllipse);

    // 元素名称列表
    treeWidgetNames = new QTreeWidget(this);
    treeWidgetNames->setColumnCount(2);
    QStringList headers;
    headers << "元素" << "元素别名";
    QTreeWidgetItem *headerItem = new QTreeWidgetItem(headers);
    treeWidgetNames->setHeaderItem(headerItem);
    //设置为多选模式
    //treeWidgetNames->setSelectionMode(QAbstractItemView::MultiSelection);

    // 元素信息列表
    treeWidgetInfo = new QTreeWidget(this);
    treeWidgetInfo->setColumnCount(2);
    QStringList Infoheaders;
    Infoheaders << "构造来源" << "来源情况";
    treeWidgetInfo->setHeaderLabels(Infoheaders);

    //工具栏
    toolBar = new QToolBar(this);
    // 后期设置 只允许 左右停靠
    toolBar->setAllowedAreas(Qt::LeftToolBarArea | Qt::RightToolBarArea);
    // 设置浮动
    toolBar->setFloatable(false);
    // 设置移动(总开关)
    toolBar->setMovable(false);
    // 工具栏中添加控件
    startButton = new QPushButton("",this);
    pauseButton = new QPushButton("",this);
    continueButton = new QPushButton("",this);
    terminateButton = new QPushButton("",this);
    QIcon icon1(":/component/eye/start.png");
    QIcon icon2(":/component/eye/stop.png");
    QIcon icon3(":/component/construct/end.png");
    QIcon icon4(":/component/construct/continue.png");
    startButton->setFixedSize((treeWidgetNames->width())/2, 40);
    pauseButton->setFixedSize((treeWidgetNames->width())/2, 40);
    continueButton->setFixedSize((treeWidgetNames->width())/2, 40);
    terminateButton->setFixedSize((treeWidgetNames->width())/2, 40);
    startButton->setIconSize(QSize(30, 30));
    pauseButton->setIconSize(QSize(25, 25));
    continueButton->setIconSize(QSize(25, 25));
    terminateButton->setIconSize(QSize(30, 30));
    startButton->setIcon(icon1);
    pauseButton->setIcon(icon3);
    terminateButton->setIcon(icon2);
    continueButton->setIcon(icon4);
    // 设置按钮没有边框，背景颜色与周围颜色一致
    startButton->setStyleSheet("QPushButton { border: none; background-color: transparent; }");
    pauseButton->setStyleSheet("QPushButton { border: none; background-color: transparent; }");
    continueButton->setStyleSheet("QPushButton { border: none; background-color: transparent; }");
    terminateButton->setStyleSheet("QPushButton { border: none; background-color: transparent; }");

    toolBar->addWidget(startButton);
    toolBar->addSeparator();
    toolBar->addWidget(pauseButton);
    toolBar->addSeparator();
    toolBar->addWidget(continueButton);
    toolBar->addSeparator();
    toolBar->addWidget(terminateButton);
    connect(continueButton, &QPushButton::clicked, this, &ElementListWidget::continueUpdate);


    // 布局
    layout->addWidget(toolBar);
    //layout->addWidget(deleteButton);
    layout->addWidget(treeWidgetNames);
    layout->addWidget(treeWidgetInfo);
    connect(treeWidgetNames, &QTreeWidget::customContextMenuRequested,
            this, &ElementListWidget::onCustomContextMenuRequested);
    treeWidgetNames->setContextMenuPolicy(Qt::CustomContextMenu);
    setFocusPolicy(Qt::StrongFocus);
    treeWidgetNames->installEventFilter(this);
    treeWidgetInfo->installEventFilter(this);
    //deleteButton->installEventFilter(this);
    toolBar->installEventFilter(this);

    connect(treeWidgetNames, &QTreeWidget::itemClicked, this, &ElementListWidget::onItemClicked);
    connect(treeWidgetNames, &QTreeWidget::itemDoubleClicked, this, &ElementListWidget::showDialog);
    connect(startButton, &QPushButton::clicked, this, &ElementListWidget::startprocess);

    //用于进度条工作
    // connect(worker, &Worker::requestSaveOperation, this, [this](int type) {
    //     switch(type) {
    //     case 0: m_pMainWin->getPWinToolWidget()->setauto(true);m_pMainWin->getPWinToolWidget()->onSaveTxt(); break;
    //     case 1: m_pMainWin->getPWinToolWidget()->onSaveWord(); break;
    //     case 2: m_pMainWin->getPWinToolWidget()->onSaveExcel(); break;
    //     case 3: m_pMainWin->getPWinToolWidget()->onSavePdf();m_pMainWin->getPWinToolWidget()->setauto(false); break;
    //     }
    // });

    // 连接树部件的双击信号
    //connect(treeWidgetNames, &QTreeWidget::itemChanged, this, &ElementListWidget::isAdd);

    // 设置状态机
    setupStateMachine();

}

void ElementListWidget::CreateEllipse(CObject*obj)
{
    QTreeWidgetItem *item = new QTreeWidgetItem(treeWidgetNames);
    item->setData(0, Qt::UserRole, QVariant::fromValue<CObject*>(obj));
    item->setText(0,obj->m_strCName);
    item->setText(1,obj->m_strAutoName);
    //QTreeWidgetItem *infoItem = new QTreeWidgetItem(treeWidgetInfo);
    //infoItem->setText(0,obj->m_strCName);
    if(obj->GetUniqueType()==enPoint){
        QIcon icon(":/component/find/point.jpg");
        item->setIcon(0, icon);
    }
    if(obj->GetUniqueType()==enLine){
        QIcon icon(":/component/find/line.jpg");
        item->setIcon(0, icon);
    }
    if(obj->GetUniqueType()==enCircle){
        QIcon icon(":/component/find/circle.jpg");
        item->setIcon(0, icon);
    }
    if(obj->GetUniqueType()==enPlane){
        QIcon icon(":/component/find/plan.jpg");
        item->setIcon(0, icon);
    }
    if(obj->GetUniqueType()==enSphere){
        QIcon icon(":/component/find/sphere.jpg");
        item->setIcon(0, icon);
    }
    if(obj->GetUniqueType()==enCone){
        QIcon icon(":/component/find/cone.jpg");
        item->setIcon(0, icon);
    }
    if(obj->GetUniqueType()==enCylinder){
        QIcon icon(":/component/find/cylinder.jpg");
        item->setIcon(0, icon);
    }
    if(obj->GetUniqueType()==enDistance){
        QIcon icon(":/component/construct/distance.png");
        item->setIcon(0, icon);
    }
    if(obj->GetUniqueType()==enPointCloud){
        QIcon icon(":/component/construct/pointCloud.png");
        item->setIcon(0, icon);
        // if(obj->GetObjectCName().contains("stand")){
        //     setModelIndex(m_pMainWin->getEntityListMgr()->getEntityList().size());
        // }
    }
    if(obj->GetUniqueType()==enAngle){
        QIcon icon(":/component/construct/angle.png");
        item->setIcon(0, icon);
    }
    if(obj->GetUniqueType()==enCuboid){
        QIcon icon(":/component/viewangle/isometric.png");
        item->setIcon(0, icon);
    }
    treeWidgetNames->scrollToBottom();
}

void ElementListWidget::onDeleteEllipse()
{
    // 1. 获取实体列表并筛选选中项
    auto& entityList = m_pMainWin->getEntityListMgr()->getEntityList();
    QVector<CEntity*> selectedEntities;
    for (CEntity* entity : entityList) {
        if (entity->getSelected()) {
            selectedEntities.append(entity);
        }
    }

    if (selectedEntities.isEmpty()) {
        return; // 无选中项则直接返回
    }

    // 2. 获取需要操作的关联列表
    auto& objectList = m_pMainWin->m_ObjectListMgr->getObjectList();
    auto& constructList = m_pMainWin->getPWinToolWidget()->getConstructEntityList();
    auto& identifyList = m_pMainWin->getPWinToolWidget()->getIdentifyEntityList();
    auto& contentMap = m_pMainWin->getpWinFileMgr()->getContentItemMap();
    auto& identifyMap = m_pMainWin->getpWinFileMgr()->getIdentifyItemMap();

    // 3. 逆序删除（避免索引错乱）
    for (int i = selectedEntities.size() - 1; i >= 0; --i) {
        CEntity* entity = selectedEntities[i];
        int index = entityList.indexOf(entity);
        if (index == -1) continue;

        CObject* obj = eleobjlist[index];

        // 特殊处理：点云类型
        if (obj->GetUniqueType() == enPointCloud) {
            CPointCloud* cloud = static_cast<CPointCloud*>(obj);
            if(pointCouldlists.contains(cloud->m_pointCloud.makeShared()))
            {
                pointCouldlists.removeOne(cloud->m_pointCloud.makeShared());
            }
            //pointCouldlists.removeOne(cloud->m_pointCloud.makeShared());
        }

        // 特殊处理：坐标系对象（临时/工件坐标系）
        QString objName = objectList[index]->GetObjectCName();
        if (objName.startsWith("临时坐标系") || objName.startsWith("工件坐标系")) {
            objectList.removeAt(index);
            continue;
        }

        // 从构造列表中移除
        for (int j = 0; j < constructList.size(); ++j) {
            if (constructList[j] == entity) {
                QString key = entity->GetObjectCName() + "  " + entity->GetObjectAutoName();
                contentMap.remove(key);
                constructList.removeAt(j);
                break;
            }
        }

        // 从识别列表中移除
        for (int j = 0; j < identifyList.size(); ++j) {
            if (identifyList[j] == entity) {
                QString key = entity->GetObjectCName() + "  " + entity->GetObjectAutoName();
                identifyMap.remove(key);
                identifyList.removeAt(j);
                break;
            }
        }

        // 从主列表中统一删除
        objectList.removeAt(index);
        entityList.removeAt(index);
        eleobjlist.removeAt(index);
    }

    // 4. 更新UI和数据
    m_pMainWin->NotifySubscribe();
    m_pMainWin->getPWinDataWidget()->getobjindex(-1);
    m_pMainWin->getPWinDataWidget()->updateinfo();
}

void ElementListWidget::onCustomContextMenuRequested(const QPoint &pos)
{
    //QTreeWidgetItem* curItem=treeWidgetNames->itemAt(pos);
    //if (!curItem) return;
    QMenu menu(this);
    QAction *action1 = menu.addAction("删除");
    QAction *action2 = menu.addAction("全部选中");
    QAction *action3 = menu.addAction("遍历列表");
    QAction *action4 = menu.addAction("设置公差");
    QAction *action7 = menu.addAction("设置别称");
    QAction *action5 = menu.addAction("显示元素信息");
    QAction *action6 = menu.addAction("关闭元素信息");
    //QAction *action8 = menu.addAction("显示标度尺");
    connect(action1, &QAction::triggered, this, &ElementListWidget::onDeleteEllipse);
    connect(action2, &QAction::triggered, this, &ElementListWidget::selectall);
    connect(action3, &QAction::triggered, this, &ElementListWidget::starttime);
    connect(action4, &QAction::triggered, this, &ElementListWidget::setTolerance);
    connect(action5, &QAction::triggered, this, &ElementListWidget::showInfotext);
    connect(action6, &QAction::triggered, this, &ElementListWidget::closeInfotext);
    connect(action7, &QAction::triggered, this, &ElementListWidget::changeName);
    //connect(action8, &QAction::triggered, this, &ElementListWidget::createrule);
    menu.exec(mapToGlobal(pos));
}

void ElementListWidget::deal_actionNew_triggered()
{

}

void ElementListWidget::updateInsertIndicatorPosition()
{

}

void ElementListWidget::upadteelementlist()
{
    treeWidgetNames->clear();
    treeWidgetInfo->clear();
    eleobjlist.clear();
    for(const auto& obj :m_pMainWin->m_ObjectListMgr->getObjectList()){
        CreateEllipse(obj);
        //obj->SetSelected(false);
        QString name=obj->GetObjectCName();
        if(name.left(5)!="临时坐标系"&&name.left(5)!="工件坐标系"){
            eleobjlist.push_back(obj);
        }
    }
}

void ElementListWidget::onItemClicked()
{
    QList<QTreeWidgetItem*> selectedItems = getSelectedItems();
    // 用于判断是否隐藏构建的元素
    QMap<QString, bool> contentMap = m_pMainWin->getpWinFileMgr()->getContentItemMap();
    QMap<QString, bool> identifyItemmap = m_pMainWin->getpWinFileMgr()->getIdentifyItemMap();

    m_pMainWin->getPWinVtkWidget()->getInteractorStyle()->CancelHighlightActors();
    if(selectedItems.size()==1){
        for(int i=0;i<m_pMainWin->getObjectListMgr()->getObjectList().size();i++){
            m_pMainWin->getObjectListMgr()->getObjectList()[i]->SetSelected(false);
        }
    }
    for(QTreeWidgetItem*item:selectedItems){
        CObject *obj = item->data(0, Qt::UserRole).value<CObject*>();
        int index=-1;
        //int entityindex=-1;
        for(int i=0;i<m_pMainWin->getObjectListMgr()->getObjectList().size();i++){
            if(m_pMainWin->getObjectListMgr()->getObjectList()[i]==obj){
                index=i;
            }
        }
        m_pMainWin->getObjectListMgr()->getObjectList()[index]->SetSelected(true);
        m_pMainWin->getPWinVtkWidget()->onHighLightActor(m_pMainWin->getEntityListMgr()->getEntityList()[index]); // 高亮列表选中的元素对应的actor
        m_pMainWin->getPWinDataWidget()->getobjindex(index);
        m_pMainWin->getPWinDataWidget()->updateinfo();
    }
    m_pMainWin->getPWinToolWidget()->updateele();
    if(selectedItems.size()==1){
        for(QTreeWidgetItem*item:selectedItems){
            CObject *obj1 = item->data(0, Qt::UserRole).value<CObject*>();
            //CEntity* ent = static_cast<CEntity*>(obj1);
            //m_pMainWin->getPWinVtkWidget()->setCentity(ent);
            ShowParent(obj1);
        }
    }
}

void ElementListWidget::setTolerance()
{
    QList<QTreeWidgetItem*> selectedItems = getSelectedItems();
    CObject *obj = selectedItems[0]->data(0, Qt::UserRole).value<CObject*>();
    if(obj->GetObjectCName().left(2)!="距离"&&obj->GetUniqueType()!=enAngle){
        QMessageBox::critical(this, "选择错误", "该元素不可以设置公差。");
        return;
    }
    dialog = new QDialog(this);
    dialog->setWindowTitle("设置公差");
    dialog->resize(200,100);
    QVBoxLayout *layout = new QVBoxLayout(dialog);
    QLabel *Up = new QLabel("上公差:");
    up = new QLineEdit();
    up->setText("0");
    up->setMaximumWidth(150);
    QLabel *Down = new QLabel("下公差:");
    down = new QLineEdit();
    down->setText("0");
    down->setMaximumWidth(150);
    updownBtn=new QPushButton("确定");
    layout->addWidget(Up, 0);
    layout->addWidget(up, 0);
    layout->addWidget(Down, 1);
    layout->addWidget(down, 1);
    layout->addWidget(updownBtn);
    dialog->setLayout(layout);
    connect(updownBtn, &QPushButton::clicked, this, &ElementListWidget::BtnClicked);
    dialog->exec();
}

void ElementListWidget::BtnClicked()
{
    double uper;
    double downer;
    bool ok;
    QList<QTreeWidgetItem*> selectedItems = getSelectedItems();
    if(selectedItems.size()!=1){
        return;
    }
    uper = up->text().toDouble(&ok);
    if (!ok) {
        QMessageBox::critical(this, "输入错误", "需要是双精度浮点类型数。");
        return;
    }
    downer = down->text().toDouble(&ok);
    if (!ok) {
        QMessageBox::critical(this, "输入错误", "需要是双精度浮点类型数。");
        return;
    }
    for(QTreeWidgetItem*item:selectedItems){
        CObject *obj = item->data(0, Qt::UserRole).value<CObject*>();
        int index=-1;
        for(int i=0;i<m_pMainWin->getObjectListMgr()->getObjectList().size();i++){
            if(m_pMainWin->getObjectListMgr()->getObjectList()[i]==obj){
                index=i;
            }
        }
        CAngle*angle;
        CDistance* dis;
        if(obj->GetUniqueType()==enAngle){
            angle = (CAngle*)m_pMainWin->getEntityListMgr()->getEntityList()[index];
            angle->setUptolerance(uper);
            angle->setUndertolerance(downer);
            angle->judge();
        }else{
            dis = dynamic_cast<CDistance*>(m_pMainWin->getEntityListMgr()->getEntityList()[index]);
            dis->setUptolerance(uper);
            dis->setUndertolerance(downer);
            dis->judge();
        }
    }
    onItemClicked();
    QMessageBox::information(this,"ok","公差设置成功");
    QMessageBox* infoBox = qobject_cast<QMessageBox*>(sender()); // 尝试获取发送者作为信息框（可选，用于直接关闭它，但通常不需要）
    if (infoBox) {
        infoBox->close(); // 通常不需要，因为信息框在用户点击后会自动关闭
    }
    dialog->close();
}

void ElementListWidget::ShowParent(CObject*obj)
{
    treeWidgetInfo->clear();
    if(obj->Form=="预制"){
        QTreeWidgetItem *infoItem = new QTreeWidgetItem(treeWidgetInfo);
        if(obj->GetUniqueType()==enPoint){
            QIcon icon(":/component/find/point.jpg");
            infoItem->setIcon(0, icon);
        }
        if(obj->GetUniqueType()==enLine){
            QIcon icon(":/component/find/line.jpg");
            infoItem->setIcon(0, icon);
        }
        if(obj->GetUniqueType()==enCircle){
            QIcon icon(":/component/find/circle.jpg");
            infoItem->setIcon(0, icon);
        }
        if(obj->GetUniqueType()==enPlane){
            QIcon icon(":/component/find/plan.jpg");
            infoItem->setIcon(0, icon);
        }
        if(obj->GetUniqueType()==enSphere){
            QIcon icon(":/component/find/sphere.jpg");
            infoItem->setIcon(0, icon);
        }
        if(obj->GetUniqueType()==enCone){
            QIcon icon(":/component/find/cone.jpg");
            infoItem->setIcon(0, icon);
        }
        if(obj->GetUniqueType()==enCylinder){
            QIcon icon(":/component/find/cylinder.jpg");
            infoItem->setIcon(0, icon);
        }
        infoItem->setText(0,obj->m_strCName);
        infoItem->setText(1,obj->Form);
        return;
    }
    if(obj->GetUniqueType()==enPointCloud){
        QTreeWidgetItem *infoItem = new QTreeWidgetItem(treeWidgetInfo);
        CPointCloud*cloud=(CPointCloud*)obj;
        if(cloud->isFileCloud){
            infoItem->setText(0,obj->m_strCName);
            infoItem->setText(1,"文件生成的点云");
        }else if(cloud->isComparsionCloud){
            infoItem->setText(0,obj->m_strCName);
            infoItem->setText(1,"对比得到的点云");
        }else if(cloud->isAlignCloud){
            infoItem->setText(0,obj->m_strCName);
            infoItem->setText(1,"对齐得到的点云");
        }
    }
    for(CObject*obj1:obj->parent){
        QTreeWidgetItem *infoItem = new QTreeWidgetItem(treeWidgetInfo);
        if(obj1->GetUniqueType()==enPoint){
            QIcon icon(":/component/construct/point.jpg");
            infoItem->setIcon(0, icon);
        }
        if(obj1->GetUniqueType()==enLine){
            QIcon icon(":/component/construct/line.jpg");
            infoItem->setIcon(0, icon);
        }
        if(obj1->GetUniqueType()==enCircle){
            QIcon icon(":/component/construct/circle.jpg");
            infoItem->setIcon(0, icon);
        }
        if(obj1->GetUniqueType()==enPlane){
            QIcon icon(":/component/construct/plan.jpg");
            infoItem->setIcon(0, icon);
        }
        if(obj1->GetUniqueType()==enSphere){
            QIcon icon(":/component/construct/sphere.jpg");
            infoItem->setIcon(0, icon);
        }
        if(obj1->GetUniqueType()==enCone){
            QIcon icon(":/component/construct/cone.jpg");
            infoItem->setIcon(0, icon);
        }
        if(obj1->GetUniqueType()==enCylinder){
            QIcon icon(":/component/construct/cylinder.jpg");
            infoItem->setIcon(0, icon);
        }
        if(obj1->GetUniqueType()==enDistance){
            QIcon icon(":/component/construct/distance.png");
            infoItem->setIcon(0, icon);
        }
        if(obj1->GetUniqueType()==enPointCloud){
            QIcon icon(":/component/construct/pointCloud.png");
            infoItem->setIcon(0, icon);
        }
        infoItem->setText(0,obj1->m_strCName);
        infoItem->setText(1,obj1->Form);
    }
}

void ElementListWidget::showInfotext()
{
    QList<QTreeWidgetItem*> selectedItems = getSelectedItems();
    QVector<CEntity*>list;
    if(selectedItems.size()==1){
        for(QTreeWidgetItem*item:selectedItems){
            CObject *obj1 = item->data(0, Qt::UserRole).value<CObject*>();
            CEntity* ent = static_cast<CEntity*>(obj1);
            /*if(obj1->GetUniqueType()==enDistance){
                CEntity* ent = static_cast<CEntity*>(obj1);
                list.push_back(ent);
            }*/
            m_pMainWin->getPWinVtkWidget()->setCentity(ent);
        }

    }
}

void ElementListWidget::closeInfotext()
{
    m_pMainWin->getPWinVtkWidget()->closeText();
}

void ElementListWidget::mousePressEvent(QMouseEvent *event)
{
    qDebug()<<"鼠标";
    // 判断点击的区域是否为空白区域
    if (treeWidgetNames->itemAt(event->pos()) == nullptr) {
        // 点击的是空白区域，取消选择
        treeWidgetNames->clearSelection();
    }
}

void ElementListWidget::setupStateMachine()
{
    // 创建状态机
    stateMachine = new QStateMachine(this);

    // 创建状态
    stoppedState = new QState();
    runningState = new QState();
    pausedState = new QState();
    continueState = new QState();

    // 配置状态间的切换
    stoppedState->addTransition(startButton, &QPushButton::clicked, runningState);
    runningState->addTransition(pauseButton, &QPushButton::clicked, pausedState);
    pausedState->addTransition(startButton, &QPushButton::clicked, runningState);
    runningState->addTransition(terminateButton, &QPushButton::clicked, stoppedState);
    pausedState->addTransition(terminateButton, &QPushButton::clicked, stoppedState);
    pausedState->addTransition(continueButton, &QPushButton::clicked, continueState);
    continueState->addTransition(startButton, &QPushButton::clicked, runningState);
    continueState->addTransition(terminateButton, &QPushButton::clicked, stoppedState);

    // 状态进入时的行为
    connect(stoppedState, &QState::entered, [this]() {
        startButton->setEnabled(true);
        pauseButton->setEnabled(false);
        continueButton->setEnabled(false);
        terminateButton->setEnabled(false);
    });

    connect(runningState, &QState::entered, [this]() {
        startButton->setEnabled(false);
        pauseButton->setEnabled(true);
        continueButton->setEnabled(false);
        terminateButton->setEnabled(true);
        // if(timer){
        //     timer->start();
        // }
        // Treelistsize=m_pMainWin->getEntityListMgr()->getEntityList().size();
    });

    connect(pausedState, &QState::entered, [this]() {
        startButton->setEnabled(true);
        pauseButton->setEnabled(false);
        continueButton->setEnabled(true);
        terminateButton->setEnabled(true);
    });

    // 添加状态到状态机
    stateMachine->addState(stoppedState);
    stateMachine->addState(runningState);
    stateMachine->addState(pausedState);
    stateMachine->addState(continueState);

    // 设置初始状态
    stateMachine->setInitialState(stoppedState);

    // 启动状态机
    stateMachine->start();

}

void ElementListWidget::continueUpdate()
{
    qDebug()<<"进行下一个";
    startupdateData(kdtree,disAndanglelist);
}

void ElementListWidget::beginStartButton()
{
    startButton->click();
}

void ElementListWidget::startprocess()
{
    qDebug()<<"点云队列数量"<<pointCouldlists.size();
    if(pointCouldlists.empty()){
        m_pMainWin->getPWinVtkPresetWidget()->setWidget("实测点云为空无法进行测量");
    }else{
        loadModelFile();
        // QTimer::singleShot(2000, []() {
        //     qDebug() << "2秒后执行的操作";
        // });
        CompareCloud();
        updateDistance();
        m_pMainWin->NotifySubscribe();
    }
}

void ElementListWidget::onAddElement(pcl::PointCloud<pcl::PointXYZRGB>::Ptr could,QString type)
{
    if (stateMachine->configuration().contains(runningState)) {
        pointCouldlists.enqueue(could);
        filetypelists.enqueue(type);
        if(isProcessing==false){
            isProcessing=true;
            loadModelFile();
            QTimer::singleShot(2000, []() {
                qDebug() << "2秒后执行的操作";
            });
            CompareCloud();
            updateDistance();
            m_pMainWin->NotifySubscribe();
        }
    }else{
        pointCouldlists.enqueue(could);
        filetypelists.enqueue(type);
    }
}

void ElementListWidget::CompareCloud()
{
    for(int i=0;i<m_pMainWin->getEntityListMgr()->getEntityList().size();i++){
        m_pMainWin->getEntityListMgr()->getEntityList()[i]->SetSelected(false);
    }
    foundmodel=false;
    for(int i=0;i<m_pMainWin->getEntityListMgr()->getEntityList().size();i++){
        if(m_pMainWin->getEntityListMgr()->getEntityList()[i]->GetUniqueType()==enPointCloud){
            CPointCloud*could=(CPointCloud*)m_pMainWin->getEntityListMgr()->getEntityList()[i];
            if(could->isModelCloud){
                foundmodel=true;
                m_pMainWin->getEntityListMgr()->getEntityList()[i]->SetSelected(true);
                break;
            }
        }
    }
    if(foundmodel==false){
        return;
    }
    bool isHaveShape=false;
    QVector<CEntity*>Shapelist;
    int CurrentMeasureindex=-1;
    for(int i=0;i<m_pMainWin->getEntityListMgr()->getEntityList().size();i++){
        if(m_pMainWin->getEntityListMgr()->getEntityList()[i]->GetUniqueType()==enCuboid){
            Shapelist.push_back(m_pMainWin->getEntityListMgr()->getEntityList()[i]);
            //m_pMainWin->getEntityListMgr()->getEntityList()[i]->SetSelected(true);
            isHaveShape=true;
        }
        if(m_pMainWin->getEntityListMgr()->getEntityList()[i]->GetUniqueType()==enPointCloud){
            if(CurrentMeasureindex!=-1){
                continue;
            }
            CPointCloud*could=(CPointCloud*)m_pMainWin->getEntityListMgr()->getEntityList()[i];
            if(could->isMeasureCloud&&could->isOver==false){
                m_pMainWin->getEntityListMgr()->getEntityList()[i]->SetSelected(true);
                CurrentMeasureindex=i;
                could->isOver=true;
            }
        }
    }

    //先执行全局对比
    if(true){
        qDebug()<<"进入对齐之前";
        m_pMainWin->getPWinVtkWidget()->onAlign();
        qDebug()<<"进入对齐之后";
        qDebug()<<CurrentMeasureindex;
        m_pMainWin->getEntityListMgr()->getEntityList()[CurrentMeasureindex]->SetSelected(false);
        m_pMainWin->getEntityListMgr()->getEntityList().back()->SetSelected(true);

        CPointCloud*could=(CPointCloud*)m_pMainWin->getEntityListMgr()->getEntityList().back();
        if(could->isAlignCloud){
            AlignCouldlists.enqueue(could->GetmyCould().makeShared());
        }
        qDebug()<<"进入对比之前";
        m_pMainWin->getPWinVtkWidget()->onCompare();
    }
    //判断是否有局部对比，若无则直接返回
    if(isHaveShape){
        //m_pMainWin->getPWinVtkWidget()->onAlign();
        //int size=m_pMainWin->getEntityListMgr()->getEntityList().size()-1;
        for(CEntity*entity:Shapelist){
            //m_pMainWin->getEntityListMgr()->getEntityList()[CurrentMeasureindex]->SetSelected(false);
            //m_pMainWin->getEntityListMgr()->getEntityList()[size]->SetSelected(true);
            entity->SetSelected(true);
            m_pMainWin->getPWinToolWidget()->onConstructPointCloud();
            entity->SetSelected(false);
        }
    }else{
        return;
    }
    for(int i=0;i<m_pMainWin->getEntityListMgr()->getEntityList().size();i++){
        m_pMainWin->getEntityListMgr()->getEntityList()[i]->SetSelected(false);
    }
    int size=0;
    for(int i=0;i<m_pMainWin->getEntityListMgr()->getEntityList().size();i++){
        CPointCloud*could=(CPointCloud*)m_pMainWin->getEntityListMgr()->getEntityList()[i];
        if(could->isCut&&could->isOver==false){
            m_pMainWin->getEntityListMgr()->getEntityList()[i]->SetSelected(true);
            could->isOver=true;
            size++;
        }
        qDebug()<<"size:"<<size;
        if(size==2){
            m_pMainWin->getPWinVtkWidget()->onCompare();
            for(int i=0;i<m_pMainWin->getEntityListMgr()->getEntityList().size();i++){
                m_pMainWin->getEntityListMgr()->getEntityList()[i]->SetSelected(false);
            }
            size=0;
        }
    }
    m_pMainWin->NotifySubscribe();
    return;
}

void ElementListWidget::updateDistance()
{
    qDebug()<<"进入updateDistance";
    // if(!foundmodel){
    //     return;
    // }
    if(timer){
        timer->stop();
        delete timer;
        timer=nullptr;
    }
    qDebug()<<"判断时间是否存在后";
    qDebug()<<"队列的大小"<<AlignCouldlists.size();
    if(AlignCouldlists.size()!=1){
        return;
    }
    kdtree.setInputCloud(AlignCouldlists.dequeue());
    pointCouldlists.dequeue();
    disAndanglelist.clear();
    for(int i=0;i<m_pMainWin->getEntityListMgr()->getEntityList().size();i++){
        if(m_pMainWin->getEntityListMgr()->getEntityList()[i]->GetObjectCName().left(2)=="距离"){
            disAndanglelist.push_back(m_pMainWin->getEntityListMgr()->getEntityList()[i]);
        }else if(m_pMainWin->getEntityListMgr()->getEntityList()[i]->GetUniqueType()==enAngle){
            disAndanglelist.push_back(m_pMainWin->getEntityListMgr()->getEntityList()[i]);
        }
    }
    if(disAndanglelist.size()==0){
        //进行图片保存，不用进度条
        m_pMainWin->getPWinToolWidget()->setauto(true);
        //m_pMainWin->getPWinToolWidget()->onSaveTxt();
        m_pMainWin->getPWinToolWidget()->onSaveWord();
        m_pMainWin->getPWinToolWidget()->onSaveExcel();
        m_pMainWin->getPWinToolWidget()->onSavePdf();
        m_pMainWin->getPWinToolWidget()->setauto(false);
        for(int i=0;i<m_pMainWin->getEntityListMgr()->getEntityList().size();i++){
            m_pMainWin->getEntityListMgr()->getEntityList()[i]->SetSelected(false);
        }
        for(int i=modelIndex;i<modelIndex+qinsSize;i++){
            m_pMainWin->getEntityListMgr()->getEntityList()[modelIndex]->SetSelected(true);
        }
        onDeleteEllipse();
        if(pointCouldlists.size()>0)
        {
            loadModelFile();
            QTimer::singleShot(2000, []() {
                qDebug() << "2秒后执行的操作";
            });
            CompareCloud();
            updateDistance();
            return;
        }else{
            isProcessing = false;
            return;
        }
    }
    qDebug()<<"进入时间开启之前";
    timer = new QTimer(this);
    list.clear();
    currentIndex=0;
    distancelistIndex=0;
    connect(timer, &QTimer::timeout, [this](){
        startupdateData(kdtree,disAndanglelist);
    });
    timer->start(1000);
    qDebug()<<"时间开始后";
}

void ElementListWidget::startupdateData(pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree,QVector<CEntity*>distancelist)
{
    qDebug()<<"时间进行1秒";
    QVector<CObject*>objlist=m_pMainWin->getObjectListMgr()->getObjectList();
    qDebug()<<"distancelistIndex："<<distancelistIndex;
    qDebug()<<"distancelist:"<<distancelist.size();
    if(distancelistIndex>distancelist.size()-1){
        if(timer){
            timer->stop();
            delete timer;
            timer=nullptr;

            // 清理旧线程
            // if (workerThread) {
            //     workerThread->quit();
            //     workerThread->wait();
            //     delete workerThread;
            //     workerThread = nullptr;
            // }

            // progressBar = new QProgressBar();
            // Qt::WindowFlags flags=Qt::Dialog|Qt::WindowCloseButtonHint;
            // progressBar->setWindowFlags(flags);//设置窗口标志
            // QFont font=QFont(tr("宋体"),10);
            // progressBar->setFont(font);//设置进度条的字体
            // progressBar->setWindowTitle(tr("Please Wait Progress Bar"));//设置进度条的窗口标题
            // progressBar->setRange(0,100);//设置进度条的数值范围，0~mTotalNum
            // progressBar->setValue(0);//设置进度条的初始值
            // progressBar->show();//显示进度条

            // workerThread = new QThread(this);
            // worker = new Worker();
            // connect(worker, &Worker::requestSaveOperation, this, [this](int type) {
            //     switch(type) {
            //     case 0: m_pMainWin->getPWinToolWidget()->setauto(true);m_pMainWin->getPWinToolWidget()->onSaveTxt(); break;
            //     case 1: m_pMainWin->getPWinToolWidget()->onSaveWord(); break;
            //     case 2: m_pMainWin->getPWinToolWidget()->onSaveExcel(); break;
            //     case 3: m_pMainWin->getPWinToolWidget()->onSavePdf();m_pMainWin->getPWinToolWidget()->setauto(false); break;
            //     }
            // },Qt::QueuedConnection);
            // worker->moveToThread(workerThread);


            // // 信号连接
            // // 使用队列连接确保跨线程安全
            // connect(workerThread, &QThread::started, worker, &Worker::doWork);
            // connect(worker, &Worker::progress, this,
            //         &ElementListWidget::updateProgress, Qt::QueuedConnection);
            // connect(worker, &Worker::finished, workerThread, &QThread::quit);
            // connect(worker, &Worker::finished, worker, &Worker::deleteLater);
            // connect(workerThread, &QThread::finished, workerThread,
            //         &QThread::deleteLater);

            // workerThread->start();

            m_pMainWin->getPWinToolWidget()->setauto(true);
            //m_pMainWin->getPWinToolWidget()->onSaveTxt();
            m_pMainWin->getPWinToolWidget()->onSaveWord();
            m_pMainWin->getPWinToolWidget()->onSaveExcel();
            m_pMainWin->getPWinToolWidget()->onSavePdf();
            m_pMainWin->getPWinToolWidget()->setauto(false);
        }
        //删除自动化打开的模型文件
        for(int i=0;i<m_pMainWin->getEntityListMgr()->getEntityList().size();i++){
            m_pMainWin->getEntityListMgr()->getEntityList()[i]->SetSelected(false);
        }
        for(int i=modelIndex;i<modelIndex+qinsSize;i++){
            m_pMainWin->getEntityListMgr()->getEntityList()[modelIndex]->SetSelected(true);
        }
        qDebug()<<"modelIndex"<<modelIndex;
        qDebug()<<"qinsSize"<<qinsSize;
        onDeleteEllipse();
        if(!pointCouldlists.empty()){
            loadModelFile();

            QTimer::singleShot(2000, []() {
                qDebug() << "2秒后执行的操作";
            });
            // QScopedPointer<QProcess> process(new QProcess);  // 自动释放
            // process->start("./compare_cloud_proc");
            // if (!process->waitForFinished(10000)) {
            //     qDebug() << "子进程崩溃或超时";
            // }
            CompareCloud();
            updateDistance();
        }else{
            isProcessing=false;
            return;
        }
    }
    if(currentIndex>distancelist[distancelistIndex]->parent.size()-1){
        UpdateDisNowFun(distancelist);
        return;
    }else if(stateMachine->configuration().contains(stoppedState)){
        timer->stop();
        delete timer;
        timer=nullptr;
        qDebug()<<"结束时间";
        return;
    }else if(stateMachine->configuration().contains(pausedState)){
        timer->stop();
        qDebug()<<"暂停时间";
        return;
    }else if(stateMachine->configuration().contains(continueState)){
        qDebug()<<"执行一次";
    }
    if(stateMachine->configuration().contains(runningState)||stateMachine->configuration().contains(continueState)){
        CObject* obj=nullptr;
        for( CObject* ob: m_pMainWin->getObjectListMgr()->getObjectList()){
            if(distancelist[distancelistIndex]->parent[currentIndex]->GetObjectCName()==ob->GetObjectCName()){
                obj=ob;
                break;
            }
        }
        currentIndex++;
        std::vector<int> pointIdxNKNSearch(1);
        std::vector<float> pointNKNSquaredDistance(1);
        pcl::PointXYZRGB searchPoint;
        if(obj->GetUniqueType()==enPoint){//点的情况一种
            CPoint*point=static_cast<CPoint*>(obj);
            searchPoint.x=point->GetPt().x;
            searchPoint.y=point->GetPt().y;
            searchPoint.z=point->GetPt().z;
            if (kdtree.nearestKSearch(searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
                int nearestIdx = pointIdxNKNSearch[0];
                pcl::PointXYZRGB nearestPoint=m_pMainWin->getpWinFileMgr()->cloudptr->points[nearestIdx];
                for(int i=0;i<objlist.size();i++){
                    if(m_pMainWin->getObjectListMgr()->getObjectList()[i]->GetObjectCName()==obj->GetObjectCName()){
                        CPoint*point1=static_cast<CPoint*>(objlist[i]);
                        CPosition pt;
                        pt.x=nearestPoint.x;pt.y=nearestPoint.y;pt.z=nearestPoint.z;
                        point1->SetPosition(pt);
                        list.push_back(point1);
                        qDebug()<<"更新距离断点1";
                        QString str=obj->GetObjectCName()+"测量完成";
                        m_pMainWin->getPWinVtkPresetWidget()->setWidget(str);
                        break;
                    }
                }
                qDebug()<<"更新距离断点2";
            }
        }else if(obj->GetUniqueType()==enPlane){//面的情况分三种
            qDebug()<<obj->parent.size();
            QVector<CObject*>planelist;
            for(CObject*obj1:m_pMainWin->getObjectListMgr()->getObjectList()){
                for(CObject* obj2:obj->parent){
                    if(obj2->GetObjectCName()==obj1->GetObjectCName()){
                        planelist.append(obj1);
                    }
                }
            }
            qDebug()<<planelist.size();
            CPlane*plane=static_cast<CPlane*>(obj);
            if(planelist.size()==3){//面由三个点构造
                QVector<CPosition>positionlist;
                for(CObject*planePt:planelist){
                    CPoint*point=static_cast<CPoint*>(planePt);
                    searchPoint.x=point->GetPt().x;
                    searchPoint.y=point->GetPt().y;
                    searchPoint.z=point->GetPt().z;
                    if (kdtree.nearestKSearch(searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
                        int nearestIdx = pointIdxNKNSearch[0];
                        pcl::PointXYZRGB nearestPoint = m_pMainWin->getpWinFileMgr()->cloudptr->points[nearestIdx];
                        CPosition pt;
                        pt.x=nearestPoint.x;
                        pt.y=nearestPoint.y;
                        pt.z=nearestPoint.z;
                        point->SetPosition(pt);
                        positionlist.push_back(pt);
                    }
                }
                PlaneConstructor constructor;
                CPlane*plane1=constructor.createPlane(positionlist[0],positionlist[1],positionlist[2]);
                plane=plane1;
                qDebug()<<"plane"<<plane1->getCenter().x;
                QString str=obj->GetObjectCName()+"测量完成";
                m_pMainWin->getPWinVtkPresetWidget()->setWidget(str);
                list.push_back(plane);
            }else if(planelist.size()==2){//面由面与面构造
                QVector<CPlane*>planeparentlist;
                for(CObject*planePlane:planelist){
                    QVector<CPosition>positionlist;
                    for(CObject*planePt:planePlane->parent){
                        CPoint*point=static_cast<CPoint*>(planePt);
                        searchPoint.x=point->GetPt().x;
                        searchPoint.y=point->GetPt().y;
                        searchPoint.z=point->GetPt().z;
                        if (kdtree.nearestKSearch(searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
                            int nearestIdx = pointIdxNKNSearch[0];
                            pcl::PointXYZRGB nearestPoint = m_pMainWin->getpWinFileMgr()->cloudptr->points[nearestIdx];
                            CPosition pt;
                            pt.x=nearestPoint.x;
                            pt.y=nearestPoint.y;
                            pt.z=nearestPoint.z;
                            point->SetPosition(pt);
                            positionlist.push_back(pt);
                        }
                    }
                    PlaneConstructor constructor;
                    CPlane*plane1=constructor.createPlane(positionlist[0],positionlist[1],positionlist[2]);
                    planePlane=plane1;
                    planeparentlist.push_back(plane1);
                }
                PlaneConstructor constructor;
                CPlane*plane1=constructor.createPlane(planeparentlist[0],planeparentlist[1]);
                plane=plane1;
                qDebug()<<"plane"<<plane1->getCenter().x;
                QString str=obj->GetObjectCName()+"测量完成";
                m_pMainWin->getPWinVtkPresetWidget()->setWidget(str);
                list.push_back(plane);
            }else if(planelist.size()==0){//面是拟合而来
                CObject*FindPoint = obj->parent[0];
                CPoint*point=static_cast<CPoint*>(FindPoint);
                searchPoint.x=point->GetPt().x;
                searchPoint.y=point->GetPt().y;
                searchPoint.z=point->GetPt().z;
                if (kdtree.nearestKSearch(searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
                    int nearestIdx = pointIdxNKNSearch[0];
                    pcl::PointXYZRGB nearestPoint = m_pMainWin->getpWinFileMgr()->cloudptr->points[nearestIdx];
                    CPosition pt;
                    pt.x=nearestPoint.x;
                    pt.y=nearestPoint.y;
                    pt.z=nearestPoint.z;
                    point->SetPosition(pt);
                }
                pcl::PointXYZRGB  Findpoint;
                Findpoint.x=point->GetPt().x;
                Findpoint.y=point->GetPt().y;
                Findpoint.z=point->GetPt().z;
                // 获取拟合用的点云指针
                auto cloudptr= m_pMainWin->getpWinFileMgr()->cloudptr;
                auto Fplane=m_pMainWin->getPWinSetDataWidget()->getPlane();
                Fplane->setRadius(plane->rad);
                Fplane->setDistance(plane->dis);
                auto could = m_pMainWin->getPWinSetDataWidget()->getPlane()->RANSAC(Findpoint,cloudptr);
                PlaneConstructor constructor;
                CPlane* newPlane;
                CPosition center;
                center.x=Fplane->getCenter()[0];
                center.y=Fplane->getCenter()[1];
                center.z=Fplane->getCenter()[2];
                QVector4D normal(Fplane->getNormal().x(),Fplane->getNormal().y(),Fplane->getNormal().z(),0);
                QVector4D direction(Fplane->getLength_Direction().x(),Fplane->getLength_Direction().y(),Fplane->getLength_Direction().z(),0);
                newPlane=constructor.createPlane(center,normal,direction,Fplane->getLength(),Fplane->getWidth());
                --(newPlane->plainCount);
                newPlane->parent.push_back(point);
                newPlane->isFind=true;
                if(newPlane==nullptr){
                    qDebug()<<"拟合平面生成错误";
                    return ;
                }
                plane = newPlane;
                QString str=obj->GetObjectCName()+"测量完成";
                m_pMainWin->getPWinVtkPresetWidget()->setWidget(str);
                list.push_back(plane);
            }
        }else if(obj->GetUniqueType()==enLine){//线的情况两种
            QVector<CPosition>positionlists;
            QVector<CPosition>positionlist;
            CLine*line=(CLine*)obj;
            if(line->parent.size()==1){//线是拟合而来
                CPoint*pointline=(CPoint*)line->parent[0];
                CPosition point = pointline->GetPt();
                positionlists.push_back(point);
            }else{//线由两个点构造
                positionlists.push_back(line->getPosition1());
                positionlists.push_back(line->getPosition2());
            }
            for(CPosition pt:positionlists){
                searchPoint.x=pt.x;
                searchPoint.y=pt.y;
                searchPoint.z=pt.z;
                if (kdtree.nearestKSearch(searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
                    int nearestIdx = pointIdxNKNSearch[0];
                    pcl::PointXYZRGB nearestPoint = m_pMainWin->getpWinFileMgr()->cloudptr->points[nearestIdx];
                    CPosition pt;
                    pt.x=nearestPoint.x;
                    pt.y=nearestPoint.y;
                    pt.z=nearestPoint.z;
                    positionlist.push_back(pt);
                }
            }
            if(line->parent.size()==1){//线是拟合而来故仅有一个点来源
                pcl::PointXYZRGB  Findpoint;
                Findpoint.x=positionlist[0].x;
                Findpoint.y=positionlist[0].y;
                Findpoint.z=positionlist[0].z;
                CPoint*pointline=(CPoint*)line->parent[0];
                pointline->SetPosition(positionlist[0]);
                // 获取拟合用的点云指针
                auto cloudptr= m_pMainWin->getpWinFileMgr()->cloudptr;
                auto Fline=m_pMainWin->getPWinSetDataWidget()->getLine();
                Fline->setDistance(line->dis);
                auto could = m_pMainWin->getPWinSetDataWidget()->getLine()->RANSAC(Findpoint,cloudptr);
                LineConstructor constructor;
                CLine* newLine;
                CPosition begin,end;
                begin.x=Fline->getBegin().x();
                begin.y=Fline->getBegin().y();
                begin.z=Fline->getBegin().z();
                end.x=Fline->getEnd().x();
                end.y=Fline->getEnd().y();
                end.z=Fline->getEnd().z();
                newLine=constructor.createLine(begin,end);
                newLine->parent.push_back(pointline);
                line=newLine;
            }else{
                line->SetPosition(positionlist[0],positionlist[1]);
            }
            QString str=obj->GetObjectCName()+"测量完成";
            m_pMainWin->getPWinVtkPresetWidget()->setWidget(str);
            list.push_back(line);
        }
        //显示左侧元素被选中效果
        for(int i=0;i<objlist.size();i++){
            if(obj->GetObjectCName()==objlist[i]->GetObjectCName()){
                m_pMainWin->getEntityListMgr()->getEntityList()[i]->SetSelected(true);
                m_pMainWin->getPWinVtkWidget()->onHighLightActor(m_pMainWin->getEntityListMgr()->getEntityList()[i]);
                QTreeWidgetItem *item = treeWidgetNames->topLevelItem(i);
                qDebug()<<"更新距离断点3";
                treeWidgetNames->setCurrentItem(item);
                break;
            }
        }
    }
}


void ElementListWidget::UpdateDisNowFun(QVector<CEntity*>distancelist)
{
    qDebug()<<list.size();
    QVector<CPoint *>position;
    QVector<CPlane*>plane;
    QVector<CLine*>line;
    for(int i=0;i<list.size();i++){
        if(list[i]->GetUniqueType()==enPoint){
            position.push_back((CPoint*)list[i]);
        }else if(list[i]->GetUniqueType()==enPlane){
            plane.push_back((CPlane*)list[i]);
        }else if(list[i]->GetUniqueType()==enLine){
            line.push_back((CLine*)list[i]);
        }
    }
    QVector<CObject*> objlist=m_pMainWin->getObjectListMgr()->getObjectList();
    for(int i=0;i<objlist.size();i++){
        if(objlist[i]->GetObjectCName()==distancelist[distancelistIndex]->GetObjectCName()){
            CAngle*angle;
            CDistance*dis;
            if(m_pMainWin->getObjectListMgr()->getObjectList()[i]->GetUniqueType()==enAngle){
                angle=dynamic_cast<CAngle*>(m_pMainWin->getObjectListMgr()->getObjectList()[i]);
            }else{
                dis=dynamic_cast<CDistance*>(m_pMainWin->getObjectListMgr()->getObjectList()[i]);
            }
            if(position.size()==2){
                dis->setbegin(position[0]->GetPt());
                dis->setend(position[1]->GetPt());
            }else if(plane.size()==1){
                dis->setbegin(position[0]->GetPt());
                dis->setplane(*plane[0]);
            }else if(plane.size()==2){
                if(objlist[i]->GetUniqueType()==enAngle){
                    AngleConstructor Constructor;
                    CAngle*angles=Constructor.createAngle(plane[0],plane[1]);
                    angle=angles;
                }else{
                    DistanceConstructor Constructor;
                    CDistance *diss=Constructor.createDistance(plane[0],plane[1]);
                    dis=diss;
                    //dis->setplane(*plane[0]);
                    //dis->setplane(*plane[1]);
                }
            }else if(line.size()==2){
                CLine line1=*line[0];
                CLine line2=*line[1];
                angle->setLine1(line1);
                angle->setLine2(line2);
            }else if(line.size()==1&&plane.size()==1){
                AngleConstructor Constructor;
                CAngle*angles=Constructor.createAngle(line[0],plane[0]);
                angle=angles;
            }
            //qDebug()<<"距离"<<dis->getdistancepoint();
            QTreeWidgetItem *item = treeWidgetNames->topLevelItem(i);
            m_pMainWin->getEntityListMgr()->getEntityList()[i]->SetSelected(true);

            m_pMainWin->getPWinVtkWidget()->onHighLightActor(m_pMainWin->getEntityListMgr()->getEntityList()[i]);
            treeWidgetNames->setCurrentItem(item);
            //m_pMainWin->getPWinToolWidget()->SaveImage(m_pMainWin->getEntityListMgr()->getEntityList()[i]);
            break;
        }
    }
    qDebug()<<"进行到距离改变";
    currentIndex=0;
    list.clear();
    QString str=distancelist[distancelistIndex]->GetObjectCName()+"测量完成";
    qDebug()<<"进行到距离改变111";
    m_pMainWin->getPWinVtkPresetWidget()->setWidget(str);
    qDebug()<<"进行到距离改变222";
    m_pMainWin->NotifySubscribe();
    qDebug()<<"进行到距离改变333";
    distancelistIndex++;
    return;
}

void ElementListWidget::isAdd()
{

}

void ElementListWidget::createrule()
{
    m_pMainWin->Createruler();
}

bool ElementListWidget::getIsProcess()
{
    return isProcessing;
}

void ElementListWidget::setModelIndex(int index)
{
    modelIndex = index;
}

void ElementListWidget::loadModelFile()
{
    int size = m_pMainWin->getEntityListMgr()->getEntityList().size();
    setModelIndex(size);
    isProcessing=true;
    m_pMainWin->peopleOpenfile = false;
    QString s = filetypelists.dequeue();
    qDebug()<<s;
    //m_pMainWin->filePathChange = m_pMainWin->modelPath+"/model"+s+"/"+s+"stand.ply";
    m_pMainWin->filePathChange = m_pMainWin->modelPath+"/model"+s+"/"+s+"stand.qins";
    m_pMainWin->openFile();
    m_pMainWin->peopleOpenfile = true;
    qinsSize = m_pMainWin->getEntityListMgr()->getEntityList().size()-size;
}

QVector<CPointCloud *> ElementListWidget::getisComparsionCloud()
{
    isComparsionCloudList.clear();
    for(int i =0;i<m_pMainWin->getEntityListMgr()->m_entityList.size();i++){
        CEntity*entity = m_pMainWin->getEntityListMgr()->m_entityList[i];
        if(entity->GetUniqueType()==enPointCloud){
            CPointCloud*could = (CPointCloud*)entity;
            if(could->isComparsionCloud){
                isComparsionCloudList.append(could);
            }
        }
    }
    return isComparsionCloudList;
}

QTreeWidget *ElementListWidget::getTreeWidgetNames()
{
    return treeWidgetNames;
}



void ElementListWidget::mouseDoubleClickEvent(QMouseEvent *event)
{
    qDebug()<<"鼠标1";
    if (event->button() == Qt::LeftButton) {
        // 获取鼠标点击位置的项
        QTreeWidgetItem *item = treeWidgetNames->itemAt(event->pos());

        if (!item) { // 如果没有项被点击，取消所有选中
            treeWidgetNames->clearSelection();
        }
    }
}

QList<QTreeWidgetItem*> ElementListWidget:: getSelectedItems(){
    return treeWidgetNames->selectedItems();
}
QVector<CObject*> ElementListWidget::getEleobjlist(){
    return eleobjlist;
}

void ElementListWidget::changeName()
{
    if(getSelectedItems().size()!=1)
    {
        QMessageBox::information(nullptr, "提示", "请选中一个元素");
        return;
    }
    QTreeWidgetItem *selectedItem=getSelectedItems()[0];
    CObject *obj = selectedItem->data(0, Qt::UserRole).value<CObject*>();
    dialog = new QDialog(this);
    dialog->setWindowTitle("设置元素别名");
    dialog->resize(200,100);
    QVBoxLayout *layout = new QVBoxLayout(dialog);
    QLabel *Name = new QLabel("别名:");
    name = new QLineEdit();
    name->setText(obj->GetObjectAutoName());
    name->setMaximumWidth(150);
    updownBtn=new QPushButton("确定");
    layout->addWidget(Name, 0);
    layout->addWidget(name, 0);
    layout->addWidget(updownBtn);
    dialog->setLayout(layout);
    connect(updownBtn, &QPushButton::clicked, this, &ElementListWidget::setAutoName);
    dialog->exec();
}

void ElementListWidget::setAutoName()
{
    int flag=0;
    bool isOpen;

    QTreeWidgetItem *selectedItem=getSelectedItems()[0];
    CObject *obj = selectedItem->data(0, Qt::UserRole).value<CObject*>();

    if(m_pMainWin->getpWinFileMgr()->getContentItemMap().contains(obj->m_strCName+"  "+obj->m_strAutoName)){
        isOpen=m_pMainWin->getpWinFileMgr()->getContentItemMap()[obj->m_strCName+"  "+obj->m_strAutoName];
        m_pMainWin->getpWinFileMgr()->getContentItemMap().remove(obj->m_strCName+"  "+obj->m_strAutoName);
        flag=1;
    }else if(m_pMainWin->getpWinFileMgr()->getIdentifyItemMap().contains(obj->m_strCName+"  "+obj->m_strAutoName)){
        isOpen=m_pMainWin->getpWinFileMgr()->getIdentifyItemMap()[obj->m_strCName+"  "+obj->m_strAutoName];
        m_pMainWin->getpWinFileMgr()->getIdentifyItemMap().remove(obj->m_strCName+"  "+obj->m_strAutoName);
        flag=2;
    }

    obj->SetObjectAutoName(name->text());

    if(flag==1){
        m_pMainWin->getpWinFileMgr()->getContentItemMap().insert(obj->m_strCName+"  "+obj->m_strAutoName,isOpen);
    }else if(flag==2){
        m_pMainWin->getpWinFileMgr()->getIdentifyItemMap().insert(obj->m_strCName+"  "+obj->m_strAutoName,isOpen);
    }

    m_pMainWin->NotifySubscribe();
    dialog->close();
}

void ElementListWidget::starttime()
{
}

void ElementListWidget::selectall()
{
    for(int i=0;i<m_pMainWin->getObjectListMgr()->getObjectList().size();i++){
        QTreeWidgetItem *item = treeWidgetNames->topLevelItem(i);
        item->setSelected(true);
        m_pMainWin->getObjectListMgr()->getObjectList()[i]->SetSelected(true);
        m_pMainWin->getEntityListMgr()->getEntityList()[i]->SetSelected(true);
    }
}

void ElementListWidget::showDialog()
{

}

void ElementListWidget::selectcommonitem()
{
    QList<QTreeWidgetItem*> selectedItems = getSelectedItems();
    if(selectedItems.size()!=1){
        return;
    }
    int ItemCount = treeWidgetNames->topLevelItemCount();
    CObject *obj = selectedItems[0]->data(0, Qt::UserRole).value<CObject*>();
    for(int i=0;i<ItemCount;i++){
        QTreeWidgetItem *item = treeWidgetNames->topLevelItem(i);
        CObject *obj1=item->data(0, Qt::UserRole).value<CObject*>();
        if(obj1->GetUniqueType()==obj->GetUniqueType()){
            item->setSelected(true);
            m_pMainWin->getObjectListMgr()->getObjectList()[i]->SetSelected(true);
        }
    }
}

void ElementListWidget::keyPressEvent(QKeyEvent *event)
{
    if (event->key() == Qt::Key_Control) {
        ctrlPressed = true; // 控制状态为按下
        treeWidgetNames->setSelectionMode(QAbstractItemView::MultiSelection); // 切换为多选模式
    }
    QWidget::keyPressEvent(event);
}

void ElementListWidget::keyReleaseEvent(QKeyEvent *event)
{
    if (event->key() == Qt::Key_Control) {
        ctrlPressed = false; // 控制状态为释放
        treeWidgetNames->setSelectionMode(QAbstractItemView::SingleSelection); // 切换为单选模式
    }
    QWidget::keyReleaseEvent(event);
}

void ElementListWidget::updateProgress(int value)
{
    if (progressBar) {
        progressBar->setValue(value);
        if (value >= 100) {
            progressBar->deleteLater(); // 安全释放
            progressBar = nullptr;
        }
    }
}

