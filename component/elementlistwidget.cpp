#include "elementlistwidget.h"
#include "mainwindow.h"
#include <QDebug>
ElementListWidget::ElementListWidget(QWidget *parent)
    : QWidget{parent}
{
    m_pMainWin=(MainWindow*)parent;
    auto *layout = new QVBoxLayout(this);

    // 元素创建按钮
    QPushButton *createButton = new QPushButton("创建元素", this);

    // 删除选中元素的按钮
    QPushButton *deleteButton = new QPushButton("删除选中元素", this);
    connect(deleteButton, &QPushButton::clicked, this, &ElementListWidget::onDeleteEllipse);

    // 元素名称列表
    treeWidgetNames = new QTreeWidget(this);
    treeWidgetNames->setHeaderLabel("元素");
    //设置为多选模式
    treeWidgetNames->setSelectionMode(QAbstractItemView::MultiSelection);

    // 元素信息列表
    treeWidgetInfo = new QTreeWidget(this);
    treeWidgetInfo->setHeaderLabel("元素信息");
    QStringList headers;
    headers << "元素：";
    treeWidgetInfo->setHeaderLabels(headers);

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
    layout->addWidget(createButton);
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

    connect(treeWidgetNames, &QTreeWidget::itemClicked, this, [this](QTreeWidgetItem*item) {
        if (item) {
            QString itemName = item->text(0);// 获取所选项的名称
            QTreeWidgetItem *parentItem = item->parent();
            int index = -1;
            if (parentItem) {
                // 遍历父项的子项，找到当前项的位置
                for (int i = 0; i < parentItem->childCount(); ++i) {
                    if (parentItem->child(i) == item) {
                        index = i;
                        break;
                    }
                }
            }
            emit itemSelected(index); // 发出自定义信号
        }
    });
}

void ElementListWidget::CreateEllipse(CObject*obj)
{
    QTreeWidgetItem *item = new QTreeWidgetItem(treeWidgetNames);
    item->setText(0,obj->m_strCName);
    QTreeWidgetItem *infoItem = new QTreeWidgetItem(treeWidgetInfo);
    infoItem->setText(0,obj->m_strCName);
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
        upadteelementlist();
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
