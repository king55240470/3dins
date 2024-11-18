#include "elementlistwidget.h"
#include "mainwindow.h"
#include "manager/filemgr.h"
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
    treeWidgetInfo->setColumnCount(2);
    treeWidgetInfo->setHeaderLabel("源");

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
    connect(treeWidgetNames, &QTreeWidget::itemDoubleClicked, this, &ElementListWidget::showDialog);
}

void ElementListWidget::CreateEllipse(CObject*obj)
{
    QTreeWidgetItem *item = new QTreeWidgetItem(treeWidgetNames);
    item->setData(0, Qt::UserRole, QVariant::fromValue<CObject*>(obj));
    item->setText(0,obj->m_strCName);
    item->setText(1,obj->m_strAutoName);
    //QTreeWidgetItem *infoItem = new QTreeWidgetItem(treeWidgetInfo);
    //infoItem->setText(0,obj->m_strCName);
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
            QVector<CEntity*> constructEntityList = m_pMainWin->getPWinToolWidget()->getConstructEntityList();//存储构建元素的列表
            QVector<CEntity*> identifyEntityList = m_pMainWin->getPWinToolWidget()->getIdentifyEntityList();//存储识别元素的列表
            if(objectList[index]->GetObjectCName().left(5)=="临时坐标系"||objectList[index]->GetObjectCName().left(5)=="工件坐标系"){
                objectList.removeAt(index);
                //pcscount--;
            }else{              
                //删除constructEntityList和contentItemMap中的元素
                for(int i=0;i<constructEntityList.size();i++){
                    if(constructEntityList[i]==entityList[entityindex]){
                        QString key=constructEntityList[i]->GetObjectCName()+"  "+constructEntityList[i]->GetObjectAutoName();
                        m_pMainWin->getpWinFileMgr()->getContentItemMap().remove(key);
                        constructEntityList.removeAt(i);
                        break;
                    }
                }

                //删除identifyEntityList和identifyItemMap中的元素
                for(int i=0;i<identifyEntityList.size();i++){
                    if(identifyEntityList[i]==entityList[entityindex]){
                        QString key=identifyEntityList[i]->GetObjectCName()+"  "+identifyEntityList[i]->GetObjectAutoName();
                        m_pMainWin->getpWinFileMgr()->getIdentifyItemMap().remove(key);
                        identifyEntityList.removeAt(i);
                        break;
                    }
                }

                objectList.removeAt(index);
                entityList.removeAt(entityindex);
                eleobjlist.removeAt(entityindex);
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
    QAction *action2 = menu.addAction("全部选中");
    QAction *action3 = menu.addAction("选中所有相同项");
    QAction *action4 = menu.addAction("设置公差");
    connect(action1, &QAction::triggered, this, &ElementListWidget::onDeleteEllipse);
    connect(action2, &QAction::triggered, this, &ElementListWidget::selectall);
    connect(action3, &QAction::triggered, this, &ElementListWidget::selectall);
    connect(action4, &QAction::triggered, this, &ElementListWidget::setTolerance);
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
    if(selectedItems.size()==1){
        for(QTreeWidgetItem*item:selectedItems){
            CObject *obj1 = item->data(0, Qt::UserRole).value<CObject*>();
            ShowParent(obj1);
        }
    }

}

void ElementListWidget::setTolerance()
{
    QList<QTreeWidgetItem*> selectedItems = getSelectedItems();
    CObject *obj = selectedItems[0]->data(0, Qt::UserRole).value<CObject*>();
    if(obj->GetObjectCName().left(2)!="距离"){
        QMessageBox::critical(this, "选择错误", "该元素不是距离类型。");
        return;
    }
    dialog = new QDialog(this);
    dialog->setWindowTitle("设置公差");
    dialog->resize(200,100);
    QVBoxLayout *layout = new QVBoxLayout(dialog);
    QLabel *Up = new QLabel("上公差:");
    up = new QLineEdit();
    up->setText("0");
    up->setMaximumWidth(150);
    QLabel *Down = new QLabel("下公差:");
    down = new QLineEdit();
    down->setText("0");
    down->setMaximumWidth(150);
    updownBtn=new QPushButton("确定");
    layout->addWidget(Up, 0);
    layout->addWidget(up, 0);
    layout->addWidget(Down, 1);
    layout->addWidget(down, 1);
    layout->addWidget(updownBtn);
    dialog->setLayout(layout);
    connect(updownBtn, &QPushButton::clicked, this, &ElementListWidget::BtnClicked);
    dialog->exec();
}

void ElementListWidget::BtnClicked()
{
    double uper;
    double downer;
    bool ok;
    QList<QTreeWidgetItem*> selectedItems = getSelectedItems();
    if(selectedItems.size()!=1){
        return;
    }
    uper = up->text().toDouble(&ok);
    if (!ok) {
        QMessageBox::critical(this, "输入错误", "圆柱体-底面中心坐标X需要是双精度浮点类型数。");
        return;
    }
    downer = down->text().toDouble(&ok);
    if (!ok) {
        QMessageBox::critical(this, "输入错误", "圆柱体-底面中心坐标Y需要是双精度浮点类型数。");
        return;
    }
    for(QTreeWidgetItem*item:selectedItems){
        CObject *obj = item->data(0, Qt::UserRole).value<CObject*>();
        if(obj->GetObjectCName().left(2)!="距离"){
            QMessageBox::critical(this,"错误","该元素不是距离类型");
            return;
        }
        int index=-1;
        for(int i=0;i<m_pMainWin->getObjectListMgr()->getObjectList().size();i++){
            if(m_pMainWin->getObjectListMgr()->getObjectList()[i]==obj){
                index=i;
            }
        }
        CDistance* dis = dynamic_cast<CDistance*>(m_pMainWin->getEntityListMgr()->getEntityList()[index]);
        CDistance* dis1 = dynamic_cast<CDistance*>(m_pMainWin->getObjectListMgr()->getObjectList()[index]);
        if(!dis||!dis1){
            qDebug()<<"dis转换失败";
            QMessageBox::critical(this, "错误", "对象转换失败，请检查。");
            return;
        }
        dis->setUptolerance(uper);
        dis->setUndertolerance(downer);
        dis->judge();
        dis1->setUptolerance(uper);
        dis1->setUndertolerance(downer);
        dis1->judge();
    }
    QMessageBox::information(this,"ok","公差设置成功");
    QMessageBox* infoBox = qobject_cast<QMessageBox*>(sender()); // 尝试获取发送者作为信息框（可选，用于直接关闭它，但通常不需要）
    if (infoBox) {
        infoBox->close(); // 通常不需要，因为信息框在用户点击后会自动关闭
    }
    dialog->close();
}

void ElementListWidget::ShowParent(CObject*obj)
{
    treeWidgetInfo->clear();
    if(obj->GetUniqueType()==enPoint){
        QTreeWidgetItem *infoItem = new QTreeWidgetItem(treeWidgetInfo);
        infoItem->setText(0,obj->m_strCName);
        infoItem->setText(1,obj->Form);
        return;
    }
    for(CObject*obj1:obj->parent){
        QTreeWidgetItem *infoItem = new QTreeWidgetItem(treeWidgetInfo);
        infoItem->setText(0,obj1->m_strCName);
        infoItem->setText(1,obj1->Form);
    }
}
QList<QTreeWidgetItem*> ElementListWidget:: getSelectedItems(){
    return treeWidgetNames->selectedItems();
}
QVector<CObject*> ElementListWidget::getEleobjlist(){
    return eleobjlist;
}

void ElementListWidget::selectall()
{
    int ItemCount = treeWidgetNames->topLevelItemCount();
    qDebug()<<ItemCount;
    for(int i=0;i<ItemCount;i++){
        QTreeWidgetItem *item = treeWidgetNames->topLevelItem(i);
        item->setSelected(true);
        m_pMainWin->getObjectListMgr()->getObjectList()[i]->SetSelected(true);
    }
}

void ElementListWidget::showDialog()
{

}

void ElementListWidget::selectcommonitem()
{
    QList<QTreeWidgetItem*> selectedItems = getSelectedItems();
    if(selectedItems.size()!=1){
        return;
    }
    int ItemCount = treeWidgetNames->topLevelItemCount();
    CObject *obj = selectedItems[0]->data(0, Qt::UserRole).value<CObject*>();
    for(int i=0;i<ItemCount;i++){
        QTreeWidgetItem *item = treeWidgetNames->topLevelItem(i);
        CObject *obj1=item->data(0, Qt::UserRole).value<CObject*>();
        if(obj1->GetUniqueType()==obj->GetUniqueType()){
            item->setSelected(true);
            m_pMainWin->getObjectListMgr()->getObjectList()[i]->SetSelected(true);
        }
    }
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


