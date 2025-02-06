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
    treeWidget->setHeaderHidden(true);//隐藏标题栏
    treeWidget->setWordWrap(true);//自动换行

    layout->addWidget(treeWidget);
    layout->setContentsMargins(0, 0, 0, 0);//消除边距
    layout->setSpacing(0);

    //设置并连接右键菜单的操作
    treeWidget->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(treeWidget, &QTreeView::customContextMenuRequested, this, &VtkPresetWidget::showContextMenu);
}

void VtkPresetWidget::setWidget(QString a){

    QDateTime currentDateTime = QDateTime::currentDateTime();  // 获取当前日期和时间
    QString time=currentDateTime.toString("yyyy-MM-dd HH:mm:ss");   // 格式化为指定的字符串格式

    QTreeWidgetItem *item = new QTreeWidgetItem(treeWidget);
    QWidget *itemWidget = new MessageItemWidget(time,a);
    treeWidget->setItemWidget(item, 0, itemWidget);

    treeWidget->addTopLevelItem(item);
}

//显示右键菜单
void VtkPresetWidget::showContextMenu(const QPoint &pos){
    //列表没有内容，不显示右键菜单
    if (treeWidget->topLevelItemCount() == 0) {
        return;
    }

    contextMenu = new QMenu(this);
    deleteAction = new QAction("清空", this);
    connect(deleteAction,&QAction::triggered,this,&VtkPresetWidget::deleteWidget);
    contextMenu->addAction(deleteAction);
    contextMenu->exec(treeWidget->mapToGlobal(pos));  // 显示右键菜单
}

void VtkPresetWidget::deleteWidget(){
    treeWidget->clear();
}




