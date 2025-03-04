#include "contralwidget.h"
#include"toolwidget.h"

ContralWidget::ContralWidget(ToolWidget* toolWidget_,QWidget *parent) : QDialog(parent) {

    toolWidget=toolWidget_;

    save_action_name_list_=toolWidget->getSaveActionNames();
    find_action_name_list_=toolWidget->getFindActionNames();
    construct_action_name_list_=toolWidget->getConstructActionNames();
    coord_action_name_list_=toolWidget->getCoordActionNames();
    view_angle_action_name_list_=toolWidget->getViewAngleActionNames();

    save_action_iconpath_list_=toolWidget->getSaveIconPath();
    find_action_iconpath_list_=toolWidget->getFindIconPath();
    construct_action_iconpath_list_=toolWidget->getConstructIconPath();
    coord_action_iconpath_list_=toolWidget->getCoordIconPath();
    view_angle_action_iconpath_list_=toolWidget->getViewAngleIconPath();


    QVBoxLayout *layout = new QVBoxLayout;
    QPushButton * ensureButton=new  QPushButton("确认",this);
    QPushButton* cancelButton=new QPushButton("取消",this);
    QHBoxLayout * layout2=new QHBoxLayout;



    fatherItems=new QTreeWidgetItem*[ToolWidget::ActionKindNum];

    setWindowTitle("图标控制");

    resize(400, 500);

    // 创建一个 QTreeWidget
    treeWidget = new QTreeWidget(this);
    treeWidget->setHeaderLabel("所有图标"); // 设置树形控件的头标签




        // 设置样式表
        treeWidget->setStyleSheet(
        "QTreeWidget {"
        "    background-color: #f3e5f5;"
        "    border: 1px solid #ce93d8;"
        "}"

        "QTreeWidget::item {"
        "    height: 32px;"
        "    padding: 7px;"
        "    font-size: 13px;"
        "    color: #4a148c;"
        "}"

        "QTreeWidget::item:selected {"
        "    background-color: #ab47bc;"
        "    color: #ffffff;"
        "}"

        "QTreeWidget::item:!selected {"
        "    background-color: #f3e5f5;"
        "    color: #4a148c;"
        "}"

        "QTreeWidget::item:hover {"
        "    background-color: #e1bee7;"
        "}"

        "QTreeWidget::indicator {"
        "    width: 17px;"
        "    height: 17px;"
        "}"
            );

    createTreeWidgetItem();


    layout->addWidget(treeWidget);
    layout2->addWidget(ensureButton);
    layout2->addWidget(cancelButton);
    layout->addLayout(layout2);
    // 确认和取消按钮的信号槽函数
    connect(cancelButton,&QPushButton::clicked,this,[this](){
        loadCheckBoxState();
        close();
    });
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


        //记录选中状态
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

         //更新到toolwidget上面
        int toolbar_index=-1;
        toolbar_index= toolWidget->addFindActions(find_action_add_list,find_action_add_num, toolbar_index);
        toolbar_index= toolWidget->addConstructActions(construct_action_add_list,construct_action_add_num, toolbar_index);
        toolbar_index= toolWidget->addSaveActions(save_action_add_list,save_action_add_num, toolbar_index);
        toolbar_index= toolWidget->addCoordActions(coord_action_add_list, coord_action_add_num, toolbar_index);
        toolbar_index= toolWidget->addViewAngleActions(view_angle_action_add_list,view_angle_action_add_num,toolbar_index);
        close();

    });

    connect(treeWidget, &QTreeWidget::itemSelectionChanged, this, &ContralWidget::onItemSelectionChanged);
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
    //添加子Item
    addActionItems(find_action_index_, toolWidget->getFindActionNum(), find_action_name_list_,find_action_iconpath_list_);
    addActionItems(construct_action_index_, toolWidget->getConstructActionNum(), construct_action_name_list_,construct_action_iconpath_list_);
    addActionItems(save_action_index_, toolWidget->getSaveActionNum(), save_action_name_list_, save_action_iconpath_list_);
    addActionItems(coord_action_index_, toolWidget->getCoordActionNum(), coord_action_name_list_,coord_action_iconpath_list_);
    addActionItems(view_angle_action_index_, toolWidget->getViewAngleActionNum(), view_angle_action_name_list_,view_angle_action_iconpath_list_);




}

void ContralWidget::addActionItems(int actionIndex, int actionNum, const QStringList *actionNameList,const QStringList *actionIconPathList) {
    for(int i = 0; i < actionNum; i++) {
        //添加子Item，设置名称和图标
        QTreeWidgetItem *iconItem = new QTreeWidgetItem(fatherItems[actionIndex], QStringList() << (*actionNameList)[i]);
        iconItem->setFlags(iconItem->flags() | Qt::ItemIsUserCheckable);
        iconItem->setCheckState(0, Qt::Checked);
        iconItem->setIcon(0, QIcon((*actionIconPathList)[i])); // 示例图标
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

void ContralWidget::mousePressEvent(QMouseEvent *event)
{
    //重载方法不成功
    QDialog::mousePressEvent(event);
}
void ContralWidget::onItemSelectionChanged()
{
    QString rootItemNames[ToolWidget::ActionKindNum]={"识别","构造","保存","坐标系","视角"};
    QList<QTreeWidgetItem*> selectedItems = treeWidget->selectedItems();
    for (QTreeWidgetItem* item : selectedItems) {
        QString name=item->text(0);
        bool IsRootItem=false;
        //检测是否是根目录的Item
        for(int i=0;i<ToolWidget::ActionKindNum;i++){
            if(name==rootItemNames[i]){
                IsRootItem=true;
                break;
            }
        }

        if(IsRootItem)break;
        //改变选中状态
        if (item->flags() & Qt::ItemIsUserCheckable) {

            Qt::CheckState state = item->checkState(0);
            if (state == Qt::Checked) {
                item->setCheckState(0, Qt::Unchecked);
            } else {
                item->setCheckState(0, Qt::Checked);
            }
            break;
        }
    }
}
