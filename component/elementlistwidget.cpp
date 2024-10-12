#include "elementlistwidget.h"
#include "mainwindow.h"
#include <QDebug>
int ElementListWidget::pcscount = 0;
ElementListWidget::ElementListWidget(QWidget *parent)
    : QWidget{parent}
{
    m_pMainWin=(MainWindow*)parent;
    auto *layout = new QVBoxLayout(this);

    // 删除选中元素的按钮
    QPushButton *deleteButton = new QPushButton("删除选中元素", this);
    connect(deleteButton, &QPushButton::clicked, this, &ElementListWidget::onDeleteEllipse);

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
    treeWidgetInfo->setHeaderLabel("元素");

    //工具栏
    toolBar = new QToolBar(this);
    // 后期设置 只允许 左右停靠
    toolBar->setAllowedAreas(Qt::LeftToolBarArea | Qt::RightToolBarArea);
    // 设置浮动
    toolBar->setFloatable(false);
    // 设置移动(总开关)
    toolBar->setMovable(false);
    // 工具栏中添加控件
    QPushButton * btn = new QPushButton("button1",this);
    QPushButton * btn1 = new QPushButton("button2",this);
    toolBar->addWidget(btn);
    toolBar->addWidget(btn1);

    // 布局
    layout->addWidget(toolBar);
    layout->addWidget(deleteButton);
    layout->addWidget(treeWidgetNames);
    layout->addWidget(treeWidgetInfo);
    connect(treeWidgetNames, &QTreeWidget::customContextMenuRequested,
            this, &ElementListWidget::onCustomContextMenuRequested);
    treeWidgetNames->setContextMenuPolicy(Qt::CustomContextMenu);
    setFocusPolicy(Qt::StrongFocus);
    treeWidgetNames->installEventFilter(this);
    treeWidgetInfo->installEventFilter(this);
    deleteButton->installEventFilter(this);
    toolBar->installEventFilter(this);

    connect(treeWidgetNames, &QTreeWidget::itemClicked, this, &ElementListWidget::onItemClicked);
}

void ElementListWidget::CreateEllipse(CObject*obj)
{
    QTreeWidgetItem *item = new QTreeWidgetItem(treeWidgetNames);
    item->setData(0, Qt::UserRole, QVariant::fromValue<CObject*>(obj));
    item->setText(0,obj->m_strCName);
    item->setText(1,obj->m_strAutoName);
    QTreeWidgetItem *infoItem = new QTreeWidgetItem(treeWidgetInfo);
    infoItem->setText(0,obj->m_strCName);
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
            auto& markList = m_pMainWin->m_EntityListMgr->getMarkList();//标记是否显示元素的列表
            if(objectList[index]->GetObjectCName().left(5)=="临时坐标系"||objectList[index]->GetObjectCName().left(5)=="工件坐标系"){
                objectList.removeAt(index);
                //pcscount--;
            }else{
                objectList.removeAt(index);
                entityList.removeAt(entityindex);
                eleobjlist.removeAt(entityindex);
                markList.removeAt(entityindex);
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
    QAction *action2 = menu.addAction("操作 2");
    QAction *action3 = menu.addAction("操作 3");
    QAction *action4 = menu.addAction("操作 4");
    connect(action1, &QAction::triggered, this, &ElementListWidget::onDeleteEllipse);
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
        m_pMainWin->getPWinDataWidget()->getobjindex(index);
        m_pMainWin->getPWinDataWidget()->updateinfo();
    }
    m_pMainWin->getPWinToolWidget()->updateele();
}
QList<QTreeWidgetItem*> ElementListWidget:: getSelectedItems(){
    return treeWidgetNames->selectedItems();
}
QVector<CObject*> ElementListWidget::getEleobjlist(){
    return eleobjlist;
}

bool ElementListWidget::eventFilter(QObject *obj, QEvent *event)
{
    if (event->type() == QEvent::MouseButtonPress) {
        QMouseEvent *mouseEvent = static_cast<QMouseEvent*>(event);
        qDebug() << "事件过滤器捕捉到鼠标按下:" << mouseEvent->button();
        // 根据需要处理事件
    }
    // 继续传递事件
    return QWidget::eventFilter(obj, event);
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

void ElementListWidget::mousePressEvent(QMouseEvent *event) {

    qDebug()<<"鼠标事件0";
    if(event->button() == Qt::RightButton){
        // 获取右键点击的坐标
        qDebug()<<"鼠标事件";
        QPoint pos = event->pos();
        //onCustomContextMenuRequested(pos);
        event->ignore();
    }else {
        // 对于其他按钮（如中键），按默认行为处理
        QWidget::mousePressEvent(event);
    }
}


