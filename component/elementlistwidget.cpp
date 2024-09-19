#include "elementlistwidget.h"
#include "mainwindow.h"
#include <QDebug>
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
    treeWidgetNames->setSelectionMode(QAbstractItemView::MultiSelection);

    // 元素信息列表
    treeWidgetInfo = new QTreeWidget(this);
    treeWidgetInfo->setHeaderLabel("元素");
    /*QStringList headers;
    headers << "元素：";
    treeWidgetInfo->setHeaderLabels(headers);*/

    //工具栏
    toolBar = new QToolBar(this);
    //addToolBar(Qt::LeftToolBarArea,toolBar);
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
    QMenu *m_menu = new QMenu(this);
    QAction* m_action1 = new QAction(tr("计划1"), this);
    QAction* m_action2 = new QAction(tr("计划2"), this);
    m_menu->addAction(m_action1);
    m_menu->addAction(m_action2);

    connect(treeWidgetNames, &QTreeWidget::itemClicked, this, &ElementListWidget::onItemClicked);
}

void ElementListWidget::CreateEllipse(CObject*obj)
{
    QTreeWidgetItem *item = new QTreeWidgetItem(treeWidgetNames);
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
        for(QTreeWidgetItem *selectedItem:selectedItems)
        {
            int row = treeWidgetNames->indexOfTopLevelItem(selectedItem);
            treeWidgetNames->takeTopLevelItem(row);
            treeWidgetInfo->takeTopLevelItem(row);
            auto& objectList = m_pMainWin->m_ObjectListMgr->getObjectList();
            objectList.erase(objectList.begin() + row);
        }
        m_pMainWin->NotifySubscribe();
    }
}

void ElementListWidget::onCustomContextMenuRequested(const QPoint &pos)
{
    QTreeWidgetItem* curItem=treeWidgetNames->itemAt(pos);
    QMenu *popMenu = new QMenu(this);
    QAction *actionNew = new QAction(tr("删除(D)"),popMenu);
    QAction *actionNew_1 = new QAction(tr("行为1"),popMenu);
    QAction *actionNew_2 = new QAction(tr("行为2"),popMenu);
    QAction *actionNew_3 = new QAction(tr("行为3"),popMenu);
    connect(actionNew, &QAction::triggered, this, &ElementListWidget::onDeleteEllipse);
    popMenu->addAction(actionNew);
    popMenu->addAction(actionNew_1);
    popMenu->addAction(actionNew_2);
    popMenu->addAction(actionNew_3);
    popMenu->exec(QCursor::pos());
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

    for(const auto& element :m_pMainWin->m_ObjectListMgr->getObjectList()){
        CreateEllipse(element);
    }
}

void ElementListWidget::onItemClicked(QTreeWidgetItem *item)
{
    int index=-1;
    index = treeWidgetNames->indexOfTopLevelItem(item);
    emit itemSelected(index); // 发出自定义信号
}
