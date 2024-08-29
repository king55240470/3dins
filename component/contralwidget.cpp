   #include "contralwidget.h"
#include "ui_contralwidget.h"
#include"toolwidget.h"
ContralWidget::ContralWidget(ToolWidget* toolWidget_,QWidget *parent) : QDialog(parent) {
    toolWidget=toolWidget_;
    memset(ActionIsChecked,true,sizeof(ActionIsChecked));
    setWindowTitle("图标控制");
    resize(300, 200);

    for(int i=0;i<59;i++)iconNames_First[i]=toolWidget->iconNames_First[i];
    for(int i=0;i<88;i++)iconNames_Second[i]=toolWidget->iconNames_Second[i];
    for(int i=0;i<6;i++)iconNames_Third[i]=toolWidget->iconNames_Third[i];
    // 创建一个垂直布局
    QVBoxLayout *layout = new QVBoxLayout;
    // 创建一个 QTreeWidget
    treeWidget = new QTreeWidget(this);
     treeWidget->setHeaderLabel("所有图标"); // 设置树形控件的头标签
    // 添加一些示例节点
    QString rootItemNames[4]={"第一栏","第二栏","第三栏","第四栏"};
    //初始化父节点
    QTreeWidgetItem* fatherItems[4];
    for(int i=0;i<4;i++){
        QTreeWidgetItem *fatherItem = new QTreeWidgetItem(treeWidget,QStringList()<<(rootItemNames[i]));
        fatherItems[i]=fatherItem;
        fatherItems[i]->setExpanded(false);
    }
    //将子节点加入父节点
    for(int i=0;i<59;i++){
        QTreeWidgetItem *iconItem = new QTreeWidgetItem(fatherItems[0],QStringList()<< iconNames_First[i]);
        iconItem->setFlags(iconItem->flags() | Qt::ItemIsUserCheckable); // 设置为复选框
        iconItem->setCheckState(0, Qt::Checked); // 设置为选中状态
    }
    for(int i=0;i<88;i++){
        QTreeWidgetItem *iconItem = new QTreeWidgetItem(fatherItems[1],QStringList()<< iconNames_First[i]);
        iconItem->setFlags(iconItem->flags() | Qt::ItemIsUserCheckable); // 设置为复选框
        iconItem->setCheckState(0, Qt::Checked); // 设置为选中状态
    }
    for(int i=0;i<6;i++){
        QTreeWidgetItem *iconItem = new QTreeWidgetItem(fatherItems[2],QStringList()<< iconNames_First[i]);
        iconItem->setFlags(iconItem->flags() | Qt::ItemIsUserCheckable); // 设置为复选框
        iconItem->setCheckState(0, Qt::Checked); // 设置为选中状态
    }
    // 将树形控件添加到布局中
    layout->addWidget(treeWidget);
    //确认和取消按钮
    QPushButton * ensureButton=new  QPushButton("确认",this);
    QPushButton* cancelButton=new QPushButton("取消",this);
    QHBoxLayout * layout2=new QHBoxLayout;
    layout2->addWidget(ensureButton);
    layout2->addWidget(cancelButton);
    layout->addLayout(layout2);
    connect(cancelButton,&QPushButton::clicked,this,[this](){loadCheckBoxState();});
    connect(ensureButton,&QPushButton::clicked,this,[this](){
        saveCheckBoxState();
        toolWidget->clear();
        for(int i=0;i<59;i++){
            if(ActionIsChecked[0][i]){
                toolWidget->addToolAction(FirstToolBarAction,iconNames_First[i]);
            }
        }
        for(int i=0;i<88;i++){
            if(ActionIsChecked[1][i])
                toolWidget->addToolAction(SecondToolBarAction,iconNames_Second[i]);
        }
        for(int i=0;i<6;i++){
            if(ActionIsChecked[2][i])
                toolWidget->addToolAction(ThirdToolBarAction,iconNames_Third[i]);
        }


    });
    setLayout(layout); // 设置窗口的布局
}
void ContralWidget::closeEvent(QCloseEvent* event){
    loadCheckBoxState();
}
void ContralWidget:: saveCheckBoxState() {
    for(int i=0;i<treeWidget->topLevelItemCount();i++){
        QTreeWidgetItem *rootWidget = treeWidget->topLevelItem(i);
        QTreeWidgetItem* item;
        for(int j=0;j<rootWidget->childCount();j++){
            item=rootWidget->child(j);
            if(Qt::Checked==item->checkState(0))
                ActionIsChecked[i][j]=true;
            else
                ActionIsChecked[i][j]=false;
        }
    }

}
void ContralWidget:: loadCheckBoxState() {
    for(int i=0;i<treeWidget->topLevelItemCount();i++){
        QTreeWidgetItem *rootWidget = treeWidget->topLevelItem(i);
        QTreeWidgetItem* item;
        for(int j=0;j<rootWidget->childCount();j++){
            item=rootWidget->child(j);
            if(ActionIsChecked[i][j])
                item->setCheckState(0,Qt::Checked);
            else
                item->setCheckState(0,Qt::Unchecked);
        }
    }
}
