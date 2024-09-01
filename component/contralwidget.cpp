#include "contralwidget.h"
#include"toolwidget.h"
ContralWidget::ContralWidget(ToolWidget* toolWidget_,QWidget *parent) : QDialog(parent) {
    toolWidget=toolWidget_;
    QVBoxLayout *layout = new QVBoxLayout;
    QPushButton * ensureButton=new  QPushButton("确认",this);
    QPushButton* cancelButton=new QPushButton("取消",this);
    QHBoxLayout * layout2=new QHBoxLayout;
    fatherItems=new QTreeWidgetItem*[ToolWidget::ToolBarCount];
    iconNames_First=toolWidget->geticonNames_First();
    iconNames_Second=toolWidget->geticonNames_Second();
    iconNames_Third=toolWidget->geticonNames_Third();
    treeWidget = new QTreeWidget(this);

    memset(ActionIsChecked,true,sizeof(ActionIsChecked));
    setWindowTitle("图标控制");
    resize(300, 200);

    treeWidget->setHeaderLabel("所有图标"); // 设置树形控件的头标签
    // 添加一些父节点
    QString rootItemNames[ToolWidget::ToolBarCount]={"第一栏","第二栏","第三栏"};
    for(int i=0;i<ToolWidget::ToolBarCount;i++){
        QTreeWidgetItem *fatherItem = new QTreeWidgetItem(treeWidget,QStringList()<<(rootItemNames[i]));
        fatherItems[i]=fatherItem;
        fatherItems[i]->setExpanded(false);
    }
    //将子节点加入父节点
    for(int i=0;i<ToolWidget::FirstToolBarActions_Num;i++){
        QTreeWidgetItem *iconItem = new QTreeWidgetItem(fatherItems[ToolWidget::FirstToolBarIndex],QStringList()<< iconNames_First[i]);
        iconItem->setFlags(iconItem->flags() | Qt::ItemIsUserCheckable); // 设置为复选框
        iconItem->setCheckState(0, Qt::Checked); // 设置为选中状态
    }
    for(int i=0;i<ToolWidget::SecondToolBarActions_Num;i++){
        QTreeWidgetItem *iconItem = new QTreeWidgetItem(fatherItems[ToolWidget::SecondToolBarIndex],QStringList()<< iconNames_Second[i]);
        iconItem->setFlags(iconItem->flags() | Qt::ItemIsUserCheckable); // 设置为复选框
        iconItem->setCheckState(0, Qt::Checked); // 设置为选中状态
    }
    for(int i=0;i<ToolWidget::ThirdToolBarActions_Num;i++){
        QTreeWidgetItem *iconItem = new QTreeWidgetItem(fatherItems[ToolWidget::ThirdToolBarIndex],QStringList()<< iconNames_Third[i]);
        iconItem->setFlags(iconItem->flags() | Qt::ItemIsUserCheckable); // 设置为复选框
        iconItem->setCheckState(0, Qt::Checked); // 设置为选中状态
    }

    layout->addWidget(treeWidget);
    layout2->addWidget(ensureButton);
    layout2->addWidget(cancelButton);
    layout->addLayout(layout2);
   // 确认和取消按钮的信号槽函数
    connect(cancelButton,&QPushButton::clicked,this,[this](){loadCheckBoxState();});
    connect(ensureButton,&QPushButton::clicked,this,[this](){
        saveCheckBoxState();
        toolWidget->clear();
        QString* Names1=new QString[ToolWidget::FirstToolBarActions_Num];
        QString* Names2=new QString[ToolWidget::SecondToolBarActions_Num];
        if(Names2==nullptr){
            qDebug()<<"Names2 is nullptr";
        }
        QString* Names3=new QString[ToolWidget::ThirdToolBarActions_Num];
        if(Names2==nullptr){
            qDebug()<<"Names3 is nullptr";
        }
        int index=-1;
        int lastToolBarIndex=-1;
        for(int i=0;i<ToolWidget::FirstToolBarActions_Num;i++){
            if(ActionIsChecked[ToolWidget::FirstToolBarIndex][i])
            {
                // toolWidget->addToolAction(FirstToolBarAction,iconNames_First[i]);
                Names1[++index]=iconNames_First[i];

            }
        }
        lastToolBarIndex=toolWidget->addToolActions(FirstToolBarAction,Names1,index+1, lastToolBarIndex);
        index=-1;
        for(int i=0;i<ToolWidget::SecondToolBarActions_Num;i++){
            if(ActionIsChecked[ToolWidget::SecondToolBarIndex][i])
            {
                // toolWidget->addToolAction(SecondToolBarAction,iconNames_Second[i]);
              Names2[++index]=iconNames_Second[i];
            }
        }


         lastToolBarIndex=toolWidget->addToolActions(SecondToolBarAction,Names2,index+1, lastToolBarIndex);
        index=-1;
        for(int i=0;i<ToolWidget::ThirdToolBarActions_Num;i++){
            if(ActionIsChecked[ToolWidget::ThirdToolBarIndex][i])
            {
                // toolWidget->addToolAction(ThirdToolBarAction,iconNames_Third[i]);
                 Names3[++index]=iconNames_Third[i];
            }
        }

         lastToolBarIndex=toolWidget->addToolActions(ThirdToolBarAction,Names3,index+1,  lastToolBarIndex);
        delete[] Names1;
        delete[] Names2;
        delete[] Names3;

    });

    setLayout(layout); // 设置窗口的布局
}
void ContralWidget::closeEvent(QCloseEvent* event){
    loadCheckBoxState();
}
void ContralWidget:: saveCheckBoxState() {
    //保存复选框状态
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
    //将复选框状态还原
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
ContralWidget::~ContralWidget(){
    if(fatherItems)delete []fatherItems;
}
