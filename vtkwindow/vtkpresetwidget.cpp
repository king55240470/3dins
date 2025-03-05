#include "vtkpresetwidget.h"
#include "mainwindow.h"

#include <QDateTime>
#include <QLabel>

VtkPresetWidget::VtkPresetWidget(QWidget *parent)
    : QWidget{parent}
{
    m_pMainWin=(MainWindow*)parent;

    layout=new QVBoxLayout(this);

    treeWidget=new QTreeWidget(this);
    // treeWidget->setSelectionMode(QAbstractItemView::NoSelection);//禁用选中
    treeWidget->setHeaderHidden(true);//隐藏标题栏
    treeWidget->setWordWrap(true);//自动换行
    treeWidget->setIndentation(0);//设置子节点前方的间距

    // 设置样式表，增大 MessageItemWidget 之间的间距
    treeWidget->setStyleSheet(
        "QTreeWidget::item { "
        //"   margin-right: 0; "
        "}"
        // "QTreeWidget{"
        // "background-color:#f0f0f0;"
        // "}"
        );


    layout->addWidget(treeWidget);
    layout->setContentsMargins(0, 0, 0, 0);//消除边距
    layout->setSpacing(0);

    //设置并连接右键菜单的操作
    treeWidget->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(treeWidget, &QTreeView::customContextMenuRequested, this, &VtkPresetWidget::showContextMenu);

    // 提升为指针类型，以便在 resizeEvent 中使用
    treeWidget->installEventFilter(this);
}

void VtkPresetWidget::setWidget(QString a){

    QDateTime currentDateTime = QDateTime::currentDateTime();  // 获取当前日期和时间
    QString time=currentDateTime.toString("yyyy-MM-dd HH:mm:ss");   // 格式化为指定的字符串格式

    QTreeWidgetItem *item = new QTreeWidgetItem(treeWidget);
    item->setSizeHint(0, QSize(0, 60)); //设置每个item的高度
    QWidget *itemWidget = new MessageItemWidget(time,a);
    itemWidget->setFixedWidth(treeWidget->width()); // 设置 MessageItemWidget 的宽度
    treeWidget->setItemWidget(item, 0, itemWidget);

    treeWidget->addTopLevelItem(item);

    treeWidget->scrollToBottom();// 滚动到底部
}

//显示右键菜单
void VtkPresetWidget::showContextMenu(const QPoint &pos){

    //列表没有内容，不显示右键菜单
    if (treeWidget->topLevelItemCount() == 0) {
        return;
    }

    // 取消选中当前项
    treeWidget->clearSelection();

    contextMenu = new QMenu(this);
    deleteAction = new QAction("清空", this);
    connect(deleteAction,&QAction::triggered,this,&VtkPresetWidget::deleteWidget);
    contextMenu->addAction(deleteAction);
    contextMenu->exec(treeWidget->mapToGlobal(pos));  // 显示右键菜单
}

void VtkPresetWidget::deleteWidget(){
    treeWidget->clear();
}

bool VtkPresetWidget::eventFilter(QObject *obj, QEvent *event)
{
    if (obj == treeWidget && event->type() == QEvent::Resize) {
        int treeWidth = treeWidget->width();
        for (int i = 0; i < treeWidget->topLevelItemCount(); ++i) {
            QTreeWidgetItem *item = treeWidget->topLevelItem(i);
            QWidget *itemWidget = treeWidget->itemWidget(item, 0);
            MessageItemWidget *messageWidget = static_cast<MessageItemWidget*>(itemWidget);
            messageWidget->updateWidth(treeWidth);
        }
    }
    return QWidget::eventFilter(obj, event);
}
