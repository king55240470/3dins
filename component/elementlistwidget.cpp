#include "elementlistwidget.h"

ElementListWidget::ElementListWidget(QWidget *parent)
    : QWidget{parent}
{
    //label=new QLabel("element list",this);
    auto *layout = new QVBoxLayout(this);
    //treeWidgetNames->setContextMenuPolicy(Qt::CustomContextMenu);
    // 元素创建按钮
    QPushButton *createButton = new QPushButton("创建元素", this);
    connect(createButton, &QPushButton::clicked, this, &ElementListWidget::onCreateEllipse);

    // 删除选中元素的按钮
    QPushButton *deleteButton = new QPushButton("删除选中元素", this);
    connect(deleteButton, &QPushButton::clicked, this, &ElementListWidget::onDeleteEllipse);

    // 元素名称列表
    treeWidgetNames = new QTreeWidget(this);
    treeWidgetNames->setHeaderLabel("元素名称");
    treeWidgetNames->setColumnCount(1);

    // 元素信息列表
    treeWidgetInfo = new QTreeWidget(this);
    treeWidgetInfo->setHeaderLabel("元素信息");
    treeWidgetInfo->setColumnCount(4);
    QStringList headers;
    headers << "ID" << "X" << "Y" << "Size";
    treeWidgetInfo->setHeaderLabels(headers);

    // 元素坐标输入
    QLineEdit *xInput = new QLineEdit(this);
    QLineEdit *yInput = new QLineEdit(this);
    QLineEdit *sizeInput = new QLineEdit(this);

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
    layout->addWidget(new QLabel("X:"));
    layout->addWidget(xInput);
    layout->addWidget(new QLabel("Y:"));
    layout->addWidget(yInput);
    layout->addWidget(new QLabel("Size:"));
    layout->addWidget(sizeInput);
    // 保存输入框的指针
    xLineEdit = xInput;
    yLineEdit = yInput;
    sizeLineEdit = sizeInput;
    connect(treeWidgetNames, &QTreeWidget::customContextMenuRequested,
            this, &ElementListWidget::onCustomContextMenuRequested);
    //treeWidgetNames->setContextMenuPolicy(Qt::CustomContextMenu);
    //QMenu *m_menu = new QMenu(this);
    //QAction* m_action1 = new QAction(tr("计划1"), this);
    //QAction* m_action2 = new QAction(tr("计划2"), this);
    //m_menu->addAction(m_action1);
    //m_menu->addAction(m_action2);
    //connect(m_action1, &QAction::triggered, this, &MainWindow::slot_checkPlan1);
    //connect(m_action2, &QAction::triggered, this, &MainWindow::slot_checkPlan2);

    // 初始化已删除ID的集合
    //deletedIds.insert(std::numeric_limits<int>::max());
    for(int i=0; i<vct.count(); ++i)
    {
        onCreateEllipse();
    }
}
void ElementListWidget::onCreateEllipse() {
    int id = getNextId();
    QString name = QString("元素%1").arg(id);

    QTreeWidgetItem *nameItem = new QTreeWidgetItem(treeWidgetNames);
    nameItem->setText(0, name);

    int x = xLineEdit->text().toInt();
    int y = yLineEdit->text().toInt();
    QString size = sizeLineEdit->text();

    QTreeWidgetItem *infoItem = new QTreeWidgetItem(treeWidgetInfo);
    infoItem->setText(0, QString::number(id));
    infoItem->setText(1, QString::number(x));
    infoItem->setText(2, QString::number(y));
    infoItem->setText(3, QString(size));

    // 可以在这里添加元素到图形界面等其他操作
}

void ElementListWidget::onDeleteEllipse() {
    QList<QTreeWidgetItem*> selectedItems = treeWidgetNames->selectedItems();
    if (!selectedItems.isEmpty()) {
        QTreeWidgetItem *selectedItem = selectedItems.first();
        int id = selectedItem->text(0).mid(2).toInt(); // 假设名称格式为"元素X"

        // 删除两个TreeWidget中的对应项
        int row = treeWidgetNames->indexOfTopLevelItem(selectedItem);
        treeWidgetNames->takeTopLevelItem(row);
        treeWidgetInfo->takeTopLevelItem(row);

        // 将ID添加到已删除ID集合
        deletedIds.insert(id);
    }
}

int ElementListWidget::getNextId() {
    // 查找下一个可用的ID
    if (!deletedIds.empty()) {
        int minDeletedId = *deletedIds.begin();
        deletedIds.erase(deletedIds.begin());
        return minDeletedId;
    }
    static int lastId = 0;
    return ++lastId;
}
void ElementListWidget::onCustomContextMenuRequested(const QPoint &pos)
{
    QTreeWidgetItem* curItem=treeWidgetNames->itemAt(pos);


    QMenu *popMenu = new QMenu(this);
    QAction *actionNew = new QAction(tr("新增(N)"),popMenu);
    connect(actionNew, &QAction::triggered, this, &ElementListWidget::deal_actionNew_triggered);
    popMenu->addAction(actionNew);
    popMenu->exec(QCursor::pos());
}

void ElementListWidget::deal_actionNew_triggered()
{
    qDebug()<<"hhhhhh";
}
