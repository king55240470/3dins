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

    // 设置头部项
    treeWidgetNames->setHeaderItem(headerItem);
    //设置为多选模式
    //treeWidgetNames->setSelectionMode(QAbstractItemView::MultiSelection);

    // 元素信息列表
    treeWidgetInfo = new QTreeWidget(this);
    treeWidgetInfo->setColumnCount(2);
    treeWidgetInfo->setHeaderLabel("源");

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
    terminateButton = new QPushButton("",this);
    QIcon icon1(":/component/construct/start.png");
    QIcon icon2(":/component/construct/stop.png");
    QIcon icon3(":/component/construct/end.png");
    startButton->setIcon(icon1);
    pauseButton->setIcon(icon3);
    terminateButton->setIcon(icon2);
    toolBar->addWidget(startButton);
    toolBar->addWidget(pauseButton);
    toolBar->addWidget(terminateButton);

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
}

void ElementListWidget::onDeleteEllipse()
{
    QList<QTreeWidgetItem*> selectedItems = treeWidgetNames->selectedItems();
    if (!selectedItems.isEmpty()) {
        /*for(QTreeWidgetItem *selectedItem:selectedItems)*/
        for(int j=selectedItems.size()-1;j>=0;j--)
        {
            QTreeWidgetItem *selectedItem=selectedItems[j];
            int index=-1;
            CObject *obj = selectedItem->data(0, Qt::UserRole).value<CObject*>();
            for(int i=0;i<m_pMainWin->getObjectListMgr()->getObjectList().size();i++){
                if(m_pMainWin->getObjectListMgr()->getObjectList()[i]==obj){
                    index=i;
                }
            }
            int entityindex=-1;
            for(int i=0;i<eleobjlist.size();i++){
                if(eleobjlist[i]==obj){
                    entityindex=i;
                }
            }
            auto& objectList = m_pMainWin->m_ObjectListMgr->getObjectList();
            auto& entityList = m_pMainWin->m_EntityListMgr->getEntityList();
            QVector<CEntity*> constructEntityList = m_pMainWin->getPWinToolWidget()->getConstructEntityList();//存储构建元素的列表
            QVector<CEntity*> identifyEntityList = m_pMainWin->getPWinToolWidget()->getIdentifyEntityList();//存储识别元素的列表
            if(objectList[index]->GetObjectCName().left(5)=="临时坐标系"||objectList[index]->GetObjectCName().left(5)=="工件坐标系"){
                objectList.removeAt(index);
                //pcscount--;
            }else{              
                //删除constructEntityList和contentItemMap中的元素
                for(int i=0;i<constructEntityList.size();i++){
                    if(constructEntityList[i]==entityList[entityindex]){
                        QString key=constructEntityList[i]->GetObjectCName()+"  "+constructEntityList[i]->GetObjectAutoName();
                        m_pMainWin->getpWinFileMgr()->getContentItemMap().remove(key);
                        constructEntityList.removeAt(i);
                        break;
                    }
                }

                //删除identifyEntityList和identifyItemMap中的元素
                for(int i=0;i<identifyEntityList.size();i++){
                    if(identifyEntityList[i]==entityList[entityindex]){
                        QString key=identifyEntityList[i]->GetObjectCName()+"  "+identifyEntityList[i]->GetObjectAutoName();
                        m_pMainWin->getpWinFileMgr()->getIdentifyItemMap().remove(key);
                        identifyEntityList.removeAt(i);
                        break;
                    }
                }

                objectList.removeAt(index);
                entityList.removeAt(entityindex);
                eleobjlist.removeAt(entityindex);
            }
        }
        m_pMainWin->NotifySubscribe();
    }
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
    QAction *action5 = menu.addAction("显示元素信息");
    QAction *action6 = menu.addAction("关闭元素信息");
    connect(action1, &QAction::triggered, this, &ElementListWidget::onDeleteEllipse);
    connect(action2, &QAction::triggered, this, &ElementListWidget::selectall);
    connect(action3, &QAction::triggered, this, &ElementListWidget::starttime);
    connect(action4, &QAction::triggered, this, &ElementListWidget::setTolerance);
    connect(action5, &QAction::triggered, this, &ElementListWidget::showInfotext);
    connect(action6, &QAction::triggered, this, &ElementListWidget::closeInfotext);
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
    if(obj->GetObjectCName().left(2)!="距离"){
        QMessageBox::critical(this, "选择错误", "该元素不是距离类型。");
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
        QMessageBox::critical(this, "输入错误", "圆柱体-底面中心坐标X需要是双精度浮点类型数。");
        return;
    }
    downer = down->text().toDouble(&ok);
    if (!ok) {
        QMessageBox::critical(this, "输入错误", "圆柱体-底面中心坐标Y需要是双精度浮点类型数。");
        return;
    }
    for(QTreeWidgetItem*item:selectedItems){
        CObject *obj = item->data(0, Qt::UserRole).value<CObject*>();
        if(obj->GetObjectCName().left(2)!="距离"){
            QMessageBox::critical(this,"错误","该元素不是距离类型");
            return;
        }
        int index=-1;
        for(int i=0;i<m_pMainWin->getObjectListMgr()->getObjectList().size();i++){
            if(m_pMainWin->getObjectListMgr()->getObjectList()[i]==obj){
                index=i;
            }
        }
        CDistance* dis = dynamic_cast<CDistance*>(m_pMainWin->getEntityListMgr()->getEntityList()[index]);
        CDistance* dis1 = dynamic_cast<CDistance*>(m_pMainWin->getObjectListMgr()->getObjectList()[index]);
        if(!dis||!dis1){
            qDebug()<<"dis转换失败";
            QMessageBox::critical(this, "错误", "对象转换失败，请检查。");
            return;
        }
        dis->setUptolerance(uper);
        dis->setUndertolerance(downer);
        dis->judge();
        dis1->setUptolerance(uper);
        dis1->setUndertolerance(downer);
        dis1->judge();
    }
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

    // 配置状态间的切换
    stoppedState->addTransition(startButton, &QPushButton::clicked, runningState);
    runningState->addTransition(pauseButton, &QPushButton::clicked, pausedState);
    pausedState->addTransition(startButton, &QPushButton::clicked, runningState);
    runningState->addTransition(terminateButton, &QPushButton::clicked, stoppedState);
    pausedState->addTransition(terminateButton, &QPushButton::clicked, stoppedState);

    // 状态进入时的行为
    connect(stoppedState, &QState::entered, [this]() {
        startButton->setEnabled(true);
        pauseButton->setEnabled(false);
        terminateButton->setEnabled(false);
    });

    connect(runningState, &QState::entered, [this]() {
        startButton->setEnabled(false);
        pauseButton->setEnabled(true);
        terminateButton->setEnabled(true);
        Treelistsize=m_pMainWin->getEntityListMgr()->getEntityList().size();
    });

    connect(pausedState, &QState::entered, [this]() {
        startButton->setEnabled(true);
        pauseButton->setEnabled(false);
        terminateButton->setEnabled(true);
    });

    // 添加状态到状态机
    stateMachine->addState(stoppedState);
    stateMachine->addState(runningState);
    stateMachine->addState(pausedState);

    // 设置初始状态
    stateMachine->setInitialState(stoppedState);

    // 启动状态机
    stateMachine->start();

}

void ElementListWidget::onAddElement()
{
    if (stateMachine->configuration().contains(runningState)) {
        updateDistance();
        CompareCloud();
        m_pMainWin->NotifySubscribe();
    }
}

void ElementListWidget::CompareCloud()
{
    for(int i=0;i<m_pMainWin->getEntityListMgr()->getEntityList().size();i++){
        CPointCloud*could=(CPointCloud*)m_pMainWin->getEntityListMgr()->getEntityList()[i];
        if(could->isModelCloud){
            m_pMainWin->getEntityListMgr()->getEntityList()[i]->SetSelected(true);
            break;
        }else{
            return;
        }
    }
    for(int i=0;i<m_pMainWin->getEntityListMgr()->getEntityList().size();i++){
        CPointCloud*could=(CPointCloud*)m_pMainWin->getEntityListMgr()->getEntityList()[i];
        if(could->isMeasureCloud){
            m_pMainWin->getEntityListMgr()->getEntityList()[i]->SetSelected(true);
            m_pMainWin->getPWinToolWidget()->setauto(true);
            m_pMainWin->getPWinVtkWidget()->onCompare();
            m_pMainWin->getPWinToolWidget()->setauto(false);
        }
    }
}

void ElementListWidget::updateDistance()
{
    qDebug()<<"进入updateDistance";
    if(timer){
        timer->stop();
        delete timer;
        timer=nullptr;
    }
    qDebug()<<"判断时间是否存在后";
    //CPointCloud*could=static_cast<CPointCloud*>(entity);
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    //kdtree.setInputCloud(could->GetmyCould().makeShared());
    kdtree.setInputCloud(m_pMainWin->getpWinFileMgr()->cloudptr);
    //QVector<CEntity*>distancelist;
    //QVector<CEntity*>anglelist;
    QVector<CEntity*>disAndanglelist;
    int distanceCount=0;
    for(int i=0;i<m_pMainWin->getEntityListMgr()->getEntityList().size();i++){
        if(m_pMainWin->getEntityListMgr()->getEntityList()[i]->GetObjectCName().left(2)=="距离"){
            disAndanglelist.push_back(m_pMainWin->getEntityListMgr()->getEntityList()[i]);
            distanceCount++;
        }else if(m_pMainWin->getEntityListMgr()->getEntityList()[i]->GetUniqueType()==enAngle){
            disAndanglelist.push_back(m_pMainWin->getEntityListMgr()->getEntityList()[i]);
        }
    }
    if(disAndanglelist.size()==0){
        return;
    }
    qDebug()<<"进入时间开启之前";
    timer = new QTimer(this);
    list.clear();
    currentIndex=0;
    distancelistIndex=0;
    connect(timer, &QTimer::timeout, [this,kdtree,disAndanglelist,distanceCount](){
        startupdateData(kdtree,disAndanglelist,distanceCount);
    });
    timer->start(1000);
    qDebug()<<"时间开始后";
}

void ElementListWidget::startupdateData(pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree,QVector<CEntity*>distancelist,int distanceCount)
{
    qDebug()<<"时间进行1秒";
    QVector<CObject*>objlist=m_pMainWin->getObjectListMgr()->getObjectList();
    if(distancelistIndex>distancelist.size()-1){
        timer->stop();
        delete timer;
        timer=nullptr;
        return;
    }
    if(currentIndex>distancelist[distancelistIndex]->parent.size()-1){
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
                treeWidgetNames->setCurrentItem(item);
                break;
            }
        }
        qDebug()<<"进行到距离改变";
        currentIndex=0;
        list.clear();
        QString str=distancelist[distancelistIndex]->GetObjectCName()+"测量完成";
        m_pMainWin->getPWinVtkPresetWidget()->setWidget(str);
        m_pMainWin->NotifySubscribe();
        m_pMainWin->getPWinToolWidget()->setauto(true);
        m_pMainWin->getPWinToolWidget()->onSaveTxt();
        m_pMainWin->getPWinToolWidget()->setauto(false);
        distancelistIndex++;
        return;
    }else if(stateMachine->configuration().contains(stoppedState)){
        timer->stop();
        delete timer;
        timer=nullptr;
        qDebug()<<"结束时间";
        return;
    }else if(stateMachine->configuration().contains(pausedState)){
        qDebug()<<"暂停时间";
        return;
    }
    if(stateMachine->configuration().contains(runningState)){
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
        if(obj->GetUniqueType()==enPoint){
            CPoint*point=static_cast<CPoint*>(obj);
            searchPoint.x=point->GetPt().x;
            searchPoint.y=point->GetPt().y;
            searchPoint.z=point->GetPt().z;
            if (kdtree.nearestKSearch(searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
                int nearestIdx = pointIdxNKNSearch[0];
                //pcl::PointXYZRGB nearestPoint = could->GetmyCould().points[nearestIdx];
                pcl::PointXYZRGB nearestPoint=m_pMainWin->getpWinFileMgr()->cloudptr->points[nearestIdx];
                for(int i=0;i<objlist.size();i++){
                    if(m_pMainWin->getObjectListMgr()->getObjectList()[i]->GetObjectCName()==obj->GetObjectCName()){
                        CPoint*point1=static_cast<CPoint*>(objlist[i]);
                        CPosition pt;
                        pt.x=nearestPoint.x;pt.y=nearestPoint.y;pt.z=nearestPoint.z;
                        point1->SetPosition(pt);
                        qDebug()<<"point1点"<<point1->GetPt().x;
                        list.push_back(point1);
                        QString str=obj->GetObjectCName()+"测量完成";
                        m_pMainWin->getPWinVtkPresetWidget()->setWidget(str);
                        break;
                    }
                }
            }
        }else if(obj->GetUniqueType()==enPlane){
            qDebug()<<obj->parent.size();
            QVector<CObject*>planelist;
            // for(CObject*obj1:obj->parent){
            //     planelist.append(obj1);
            // }
            for(CObject*obj1:m_pMainWin->getObjectListMgr()->getObjectList()){
                for(CObject* obj2:obj->parent){
                    if(obj2->GetObjectCName()==obj1->GetObjectCName()){
                        planelist.append(obj1);
                    }
                }
            }
            qDebug()<<planelist.size();
            CPlane*plane=static_cast<CPlane*>(obj);
            if(planelist.size()==3){
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
            }
        }else if(obj->GetUniqueType()==enLine){
            QVector<CPosition>positionlists;
            QVector<CPosition>positionlist;
            CLine*line=(CLine*)obj;
            positionlists.push_back(line->getPosition1());
            positionlists.push_back(line->getPosition2());
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
            line->SetPosition(positionlist[0],positionlist[1]);
            QString str=obj->GetObjectCName()+"测量完成";
            m_pMainWin->getPWinVtkPresetWidget()->setWidget(str);
            list.push_back(line);
        }
        for(int i=0;i<objlist.size();i++){
            if(obj->GetObjectCName()==objlist[i]->GetObjectCName()){
                QTreeWidgetItem *item = treeWidgetNames->topLevelItem(i);
                treeWidgetNames->setCurrentItem(item);
                break;
            }
        }
    }
}

void ElementListWidget::isAdd()
{
    int Nowlistsize=m_pMainWin->getEntityListMgr()->getEntityList().size();
    int Nowlistsizes=m_pMainWin->getpWinFileMgr()->getMeasuredFileMap().size();
    if(Nowlistsize>Treelistsize){
        Treelistsize=Nowlistsize;
        onAddElement();
    }else if(Nowlistsize<Treelistsize){
        Treelistsize=Nowlistsize;
    }
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

void ElementListWidget::starttime()
{
    currentIndex=0;
    timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &ElementListWidget::onAddElement);
    timer->start(1000);
}

void ElementListWidget::selectall()
{
    /*int ItemCount = treeWidgetNames->topLevelItemCount();
    qDebug()<<ItemCount;
    for(int i=0;i<ItemCount;i++){
        QTreeWidgetItem *item = treeWidgetNames->topLevelItem(i);
        item->setSelected(true);
        m_pMainWin->getObjectListMgr()->getObjectList()[i]->SetSelected(true);
    }*/

    if(currentIndex < treeWidgetNames->topLevelItemCount()) {
        QTreeWidgetItem *item = treeWidgetNames->topLevelItem(currentIndex);
        treeWidgetNames->setCurrentItem(item);
        currentIndex++;
    }else{
        timer->stop();
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


