#include "contralwidget.h"
#include"toolwidget.h"
ContralWidget::ContralWidget(ToolWidget* toolWidget_,QWidget *parent) : QDialog(parent) {

    toolWidget=toolWidget_;

    save_action_name_list_=toolWidget->getSaveActionNames();
    find_action_name_list_=toolWidget->getFindActionNames();
    construct_action_name_list_=toolWidget->getConstructActionNames();
    coord_action_name_list_=toolWidget->getCoordActionNames();
    view_angle_action_name_list_=toolWidget->getViewAngleActionNames();


    QVBoxLayout *layout = new QVBoxLayout;
    QPushButton * ensureButton=new  QPushButton("确认",this);
    QPushButton* cancelButton=new QPushButton("取消",this);
    QHBoxLayout * layout2=new QHBoxLayout;



    fatherItems=new QTreeWidgetItem*[ToolWidget::ActionKindNum];

    setWindowTitle("图标控制");

    resize(300, 200);

    // 创建一个 QTreeWidget
    treeWidget = new QTreeWidget(this);
    treeWidget->setHeaderLabel("所有图标"); // 设置树形控件的头标签

    createTreeWidgetItem();


    layout->addWidget(treeWidget);
    layout2->addWidget(ensureButton);
    layout2->addWidget(cancelButton);
    layout->addLayout(layout2);
    // 确认和取消按钮的信号槽函数
    connect(cancelButton,&QPushButton::clicked,this,[this](){loadCheckBoxState();});
    connect(ensureButton,&QPushButton::clicked,this,[this](){
        saveCheckBoxState();
        toolWidget->clearToolWidget();
        QStringList save_action_add_list;
        QStringList find_action_add_list;
        QStringList construct_action_add_list;
        QStringList coord_action_add_list;
        QStringList view_angle_action_add_list;

        int save_action_add_num=0;
        int find_action_add_num=0;
        int construct_action_add_num=0;
        int coord_action_add_num=0;
        int view_angle_action_add_num=0;

        for(int i=0;i<toolWidget->getFindActionNum();i++){
            if(action_is_checked_[find_action_index_][i]) {
                find_action_add_list<<(*find_action_name_list_)[i];
                find_action_add_num++;
            }
        }
        for(int i=0;i<toolWidget->getConstructActionNum();i++){
            if(action_is_checked_[construct_action_index_][i]) {
                construct_action_add_list<<(*construct_action_name_list_)[i];
                construct_action_add_num++;
            }
        }
        for(int i=0;i<toolWidget->getSaveActionNum();i++){
            if(action_is_checked_[save_action_index_][i]) {
                save_action_add_list<<(*save_action_name_list_)[i];
                save_action_add_num++;
            }
        }
        for(int i=0;i<toolWidget->getCoordActionNum();i++){
            if(action_is_checked_[coord_action_index_][i]) {
                coord_action_add_list<<(*coord_action_name_list_)[i];
                coord_action_add_num++;
            }
        }
        for(int i=0;i<toolWidget->getViewAngleActionNum();i++){
            if(action_is_checked_[view_angle_action_index_][i]) {
                view_angle_action_add_list<<(*view_angle_action_name_list_)[i];
                view_angle_action_add_num++;
            }
        }

        int toolbar_index=-1;
        toolbar_index= toolWidget->addFindActions(find_action_add_list,find_action_add_num, toolbar_index);
        toolbar_index= toolWidget->addConstructActions(construct_action_add_list,construct_action_add_num, toolbar_index);
        toolbar_index= toolWidget->addSaveActions(save_action_add_list,save_action_add_num, toolbar_index);
        toolbar_index= toolWidget->addCoordActions(coord_action_add_list, coord_action_add_num, toolbar_index);
        toolbar_index= toolWidget->addViewAngleActions(view_angle_action_add_list,view_angle_action_add_num,toolbar_index);


    });

    setLayout(layout); // 设置窗口的布局
}
void ContralWidget::createTreeWidgetItem(){
    // 添加一些父节点
    QString rootItemNames[ToolWidget::ActionKindNum]={"识别","构造","保存","坐标系","视角"};
    for(int i=0;i<ToolWidget::ActionKindNum;i++){
        QTreeWidgetItem *fatherItem = new QTreeWidgetItem(treeWidget,QStringList()<<(rootItemNames[i]));

        fatherItems[i]=fatherItem;
        fatherItems[i]->setExpanded(false);
    }

    //将子节点加入父节点
    for(int i=0;i<toolWidget->getFindActionNum();i++){

        QTreeWidgetItem *iconItem = new QTreeWidgetItem(fatherItems[find_action_index_],QStringList()<<(* find_action_name_list_)[i]);

        iconItem->setFlags(iconItem->flags() | Qt::ItemIsUserCheckable); // 设置为复选框
        iconItem->setCheckState(0, Qt::Checked); // 设置为选中状态
    }
    for(int i=0;i<toolWidget->getConstructActionNum();i++){
        QTreeWidgetItem *iconItem = new QTreeWidgetItem(fatherItems[construct_action_index_],QStringList()<< (*construct_action_name_list_)[i]);
        iconItem->setFlags(iconItem->flags() | Qt::ItemIsUserCheckable); // 设置为复选框
        iconItem->setCheckState(0, Qt::Checked); // 设置为选中状态
    }
    for(int i=0;i<toolWidget->getSaveActionNum();i++){
        QTreeWidgetItem *iconItem = new QTreeWidgetItem(fatherItems[save_action_index_],QStringList()<< (*save_action_name_list_)[i]);
        iconItem->setFlags(iconItem->flags() | Qt::ItemIsUserCheckable); // 设置为复选框
        iconItem->setCheckState(0, Qt::Checked); // 设置为选中状态
    }
    for(int i=0;i<toolWidget->getCoordActionNum();i++){
        QTreeWidgetItem *iconItem = new QTreeWidgetItem(fatherItems[coord_action_index_],QStringList()<< (*coord_action_name_list_)[i]);
        iconItem->setFlags(iconItem->flags() | Qt::ItemIsUserCheckable); // 设置为复选框
        iconItem->setCheckState(0, Qt::Checked); // 设置为选中状态
    }
    for(int i=0;i<toolWidget->getViewAngleActionNum();i++){
        QTreeWidgetItem *iconItem = new QTreeWidgetItem(fatherItems[view_angle_action_index_],QStringList()<< (*view_angle_action_name_list_)[i]);
        iconItem->setFlags(iconItem->flags() | Qt::ItemIsUserCheckable); // 设置为复选框
        iconItem->setCheckState(0, Qt::Checked); // 设置为选中状态
    }


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
                action_is_checked_[i][j]=true;
            else
                action_is_checked_[i][j]=false;
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
            if(action_is_checked_[i][j])
                item->setCheckState(0,Qt::Checked);
            else
                item->setCheckState(0,Qt::Unchecked);
        }
    }
}
ContralWidget::~ContralWidget(){
    if(fatherItems)delete []fatherItems;
}
