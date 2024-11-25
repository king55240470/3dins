#include "filemanagerwidget.h"
#include "mainwindow.h"
#include "manager/filemgr.h"
#include "geometry/centity.h"
#include "vtkwindow/vtkwidget.h"

#include <QMainWindow>
#include <QDebug>
#include <QIcon>

FileManagerWidget::FileManagerWidget(QWidget *parent)
    : QWidget{parent}
{
    m_pMainWin=(MainWindow*)parent;

    layout=new QVBoxLayout(this);

    compareBtn=new QPushButton("对比");

    filetree=new QTreeView(this);
    filetree->setEditTriggers(QAbstractItemView::NoEditTriggers);
    //设置QTreeView的字体大小
    int width = filetree->width();
    int height = filetree->height();
    int newFontSize = qMin(width/2, height/3);
    QFont font = this->font();
    font.setPointSize(newFontSize);
    filetree->setFont(font);

    delegate = new ButtonDelegate(filetree);
    filetree->setItemDelegate(delegate);

    model=new QStandardItemModel(this);
    model->setHorizontalHeaderLabels(QStringList()<<"文件管理器");

    rootItem = model->invisibleRootItem();

    modelFile=new QStandardItem("打开的模型文件");
    rootItem->appendRow(modelFile);

    measuredFile = new QStandardItem("打开的实测文件");
    rootItem->appendRow(measuredFile);

    contentItem = new QStandardItem("构建的内容");
    rootItem->appendRow(contentItem);

    identifyItem = new QStandardItem("识别的内容");
    rootItem->appendRow(identifyItem);

    filetree->setModel(model);

    layout->addWidget(compareBtn);
    layout->addWidget(filetree);
    layout->setContentsMargins(0, 0, 0, 0);//消除边距
    layout->setSpacing(0);

    connect(filetree, &QTreeView::clicked, this, &FileManagerWidget::getItem);
    connect(delegate, &ButtonDelegate::buttonClicked, this, &FileManagerWidget::changePlay);

    //设置并连接右键菜单的操作
    filetree->setContextMenuPolicy(Qt::CustomContextMenu);//设置了上下文菜单策略为Qt::CustomContextMenu，这意味着当用户在该视图上右键点击时，不会显示默认的上下文菜单，而是会触发一个信号，允许开发者自定义要显示的菜单
    connect(filetree, &QTreeView::customContextMenuRequested, this, &FileManagerWidget::showContextMenu);//当用户右键点击filetree中的任意位置时，customContextMenuRequested信号会被触发，Qt将自动调用FileManagerWidget的showContextMenu函数

    //对比按钮
    connect(compareBtn,&QPushButton::clicked,this, [&](){
        m_pMainWin->getPWinVtkWidget()->onCompare();
    });
}

void FileManagerWidget::openModelFile(QString fileName,QString filePath){
    QStandardItem *newFileItem = new QStandardItem(fileName);
    newFileItem->setData(filePath, Qt::UserRole);//在QTreeView的子节点中存储文件路径（Qt::UserRole用来存储用户第一个定义的数据：文件路径）
    newFileItem->setData(true, Qt::UserRole+1); //设置初始数据以便按钮绘制（Qt::UserRole+1用来存储用户第二个定义的数据：按钮状态）
    modelFile->appendRow(newFileItem);

    //在modelFileMap中添加添加新文件，并分配新的cloud
    m_pMainWin->getpWinFileMgr()->getModelFileMap().insert(filePath, true);
    auto cloud = m_pMainWin->getPointCloudListMgr()->CreateCloudFromFile(filePath);
    m_pMainWin->getPWinToolWidget()->addToList(cloud);

    // 给拟合的临时点云指针赋值
    auto newcloud = new pcl::PointCloud<pcl::PointXYZRGB>(cloud->m_pointCloud);

    auto tmpCloud=m_pMainWin->getPointCloudListMgr()->getTempCloud();
    pcl::copyPointCloud(*newcloud, tmpCloud);

    m_pMainWin->getpWinFileMgr()->cloudptr = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(newcloud);
    if(!m_pMainWin->getpWinFileMgr()->cloudptr){
        qDebug() << "拟合用的点云指针为空!";
    }
    m_pMainWin->NotifySubscribe();
}

void FileManagerWidget::openMeasuredFile(QString fileName,QString filePath){
    QStandardItem *newFileItem = new QStandardItem(fileName);
    newFileItem->setData(filePath, Qt::UserRole);//在QTreeView的子节点中存储文件路径
    newFileItem->setData(true, Qt::UserRole+1);
    measuredFile->appendRow(newFileItem);

    //在measuredFileMap中添加新文件，并分配新的cloud
    m_pMainWin->getpWinFileMgr()->getMeasuredFileMap().insert(filePath, true);
    auto cloud = m_pMainWin->getPointCloudListMgr()->CreateCloudFromFile(filePath);
    m_pMainWin->getPWinToolWidget()->addToList(cloud);

    // 给拟合的临时点云指针赋值
    auto newcloud = new pcl::PointCloud<pcl::PointXYZRGB>(cloud->m_pointCloud);
    m_pMainWin->getpWinFileMgr()->cloudptr = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(newcloud);
    m_pMainWin->NotifySubscribe();
}

void FileManagerWidget::createContentItem(){
    //删除contentItem中的所有子项
    int childCount = contentItem->rowCount(); // 获取子项数量
    for (int i=childCount-1;i>=0;i--) {
        contentItem->removeRow(i); // 删除子项
    }

    //添加子项
    //获取所有键值
    QList<QString> keys = m_pMainWin->getpWinFileMgr()->getContentItemMap().keys();
    // 遍历所有的键
    for (const QString &key : keys) {
        QStandardItem *newContentItem = new QStandardItem(key);
        newContentItem->setData(key, Qt::UserRole);
        newContentItem->setData(m_pMainWin->getpWinFileMgr()->getContentItemMap()[key], Qt::UserRole+1);
        contentItem->appendRow(newContentItem);

        qDebug() << "Key:" << key << ", Value:" << m_pMainWin->getpWinFileMgr()->getContentItemMap()[key];
    }
}

void FileManagerWidget::createIdentifyItem(){
    //删除identifyItem中的所有子项
    int childCount = identifyItem->rowCount(); // 获取子项数量
    for (int i=childCount-1;i>=0;i--) {
        identifyItem->removeRow(i); // 删除子项
    }

    //添加子项
    //获取所有键值
    QList<QString> keys = m_pMainWin->getpWinFileMgr()->getIdentifyItemMap().keys();
    // 遍历所有的键
    for (const QString &key : keys) {
        QStandardItem *newIentifyItem = new QStandardItem(key);
        newIentifyItem->setData(key, Qt::UserRole);
        newIentifyItem->setData(m_pMainWin->getpWinFileMgr()->getIdentifyItemMap()[key], Qt::UserRole+1);
        identifyItem->appendRow(newIentifyItem);

        qDebug() << "Key:" << key << ", Value:" << m_pMainWin->getpWinFileMgr()->getIdentifyItemMap()[key];
    }
}

// void FileManagerWidget::createPresetOpen(CEntity *obj){
//     // QString str=obj->m_strCName+"  "+obj->m_strAutoName;
//     // QStandardItem *newObjItem = new QStandardItem(str);
//     // newObjItem->setData(true, Qt::UserRole+1);
//     // newObjItem->setData(QVariant::fromValue<CEntity*>(obj),Qt::UserRole+2);
//     // contentItem->appendRow(newObjItem);
// }

// void FileManagerWidget::createPresetClose(CEntity *obj){
//     // QString str=obj->m_strCName+"  "+obj->m_strAutoName;
//     // QStandardItem *newObjItem = new QStandardItem(str);
//     // newObjItem->setData(false, Qt::UserRole+1);
//     // newObjItem->setData(QVariant::fromValue<CEntity*>(obj),Qt::UserRole+2);
//     // contentItem->appendRow(newObjItem);
// }

//显示右键菜单
void FileManagerWidget::showContextMenu(const QPoint &pos){
    selectedIndex = filetree->indexAt(pos);  // 获取点击位置的索引
    selectedItem = model->itemFromIndex(selectedIndex);  // 获取QStandardItem
    if (isChildOf(selectedItem, modelFile) || isChildOf(selectedItem, measuredFile)) {
        qDebug() << "进入删除操作";
        // qDebug()<<selectedIndex;
        // qDebug()<<selectedItem;
        contextMenu = new QMenu(this);
        deleteAction = new QAction("删除文件", this);
        createAction = new QAction("重新生成", this);
        connect(deleteAction,&QAction::triggered,this,&FileManagerWidget::deleteFile);
        connect(createAction,&QAction::triggered,this,&FileManagerWidget::redrawCloudEntity);
        contextMenu->addAction(deleteAction);
        contextMenu->addAction(createAction);
        contextMenu->exec(filetree->mapToGlobal(pos));  // 显示右键菜单
    }
}

//从项目中删除文件
void FileManagerWidget::deleteFile(){
    QString filePath = selectedItem->data(Qt::UserRole).toString();//从子节点中获取文件路径

    if(isChildOf(selectedItem, modelFile)){
        m_pMainWin->getpWinFileMgr()->getModelFileMap().remove(filePath);
        // 删除文件在点云map中的记录
        m_pMainWin->getPointCloudListMgr()->DeleteFileCloud(filePath);

        bool removed=model->removeRow(selectedIndex.row(),selectedIndex.parent());
        if(removed){
            qDebug()<<"文件被删除";
        }else{
            qDebug()<<"文件没有被删除";
        }

        qDebug() <<"删除操作后还剩余的ModelFile";
        QMap<QString, bool>& fileList = m_pMainWin->getpWinFileMgr()->getModelFileMap();
        QMap<QString, bool>::const_iterator it;
        for (it = fileList.cbegin(); it != fileList.cend(); ++it) {
            qDebug() << "Key:" << it.key() << ", Value:" << it.value();
        }
    }else if(isChildOf(selectedItem, measuredFile)){
        m_pMainWin->getpWinFileMgr()->getMeasuredFileMap().remove(filePath);
        bool removed=model->removeRow(selectedIndex.row(),selectedIndex.parent());
        if(removed){
            qDebug()<<"文件被删除";
        }else{
            qDebug()<<"文件没有被删除";
        }

        qDebug() <<"删除操作后还剩余的MeasuredFile";
        QMap<QString, bool>& fileList = m_pMainWin->getpWinFileMgr()->getMeasuredFileMap();
        QMap<QString, bool>::const_iterator it;
        for (it = fileList.cbegin(); it != fileList.cend(); ++it) {
            qDebug() << "Key:" << it.key() << ", Value:" << it.value();
        }
    }else{
        qDebug()<<"选择的不是文件";
    }

    m_pMainWin->NotifySubscribe();

    //注释部分是彻底从电脑上删除文件，而不是从项目中把文件移除
    // QFileInfo fileInfo(filePath);
    // if(fileInfo.isFile()){//判断点击的是否为文件
    //     QFile file(filePath);
    //     if(file.remove()){
    //         qDebug()<<"文件被删除";
    //     }else{
    //         qDebug()<<"文件没有被删除";
    //     }
    // }else{
    //     qDebug()<<"选择的不是文件";
    // }
}

// 重新加载点云
void FileManagerWidget::redrawCloudEntity()
{
    QString filePath = selectedItem->data(Qt::UserRole).toString();//从子节点中获取文件路径
    // 由选中的文件重新加载点云，并存入entitylist
    m_pMainWin->getPWinToolWidget()->addToList(
        m_pMainWin->getPointCloudListMgr()->CreateCloudFromFile(filePath));
    m_pMainWin->NotifySubscribe();
}

//点击文件名获得文件路径&按钮状态
void FileManagerWidget::getItem(const QModelIndex &index){
    if (!index.isValid()) {
        return; // 确保索引有效
    }
    QStandardItem *item = model->itemFromIndex(index);//获取被点击的项
    QString filePath = item->data(Qt::UserRole).toString();//从子节点中获取文件路径
    bool isBtnOpen = item->data(Qt::UserRole+1).toBool();//从子节点中获取按钮状态
    QString status = isBtnOpen ? "按钮状态: 显示文件内容" : "按钮状态: 不显示文件内容";
    if (item && filePath!="") {
        qDebug() << filePath;
        qDebug() << status;
    } else {
        qDebug() << status;
    }
}

void FileManagerWidget::changePlay(const QModelIndex &index){
    QStandardItem* childItem = model->itemFromIndex(index);
    if(isChildOf(childItem, modelFile)){
        changeModelFile(index);
    }else if(isChildOf(childItem, measuredFile)){
        changeMeasuredFile(index);
    }else if(isChildOf(childItem, contentItem)){
        changeContentItem(index);
    }else if(isChildOf(childItem, identifyItem)){
        changeIdentifyItem(index);
    }
}

void FileManagerWidget::changeModelFile(const QModelIndex &index){
    QStandardItem *item = model->itemFromIndex(index);
    QString filePath = item->data(Qt::UserRole).toString();
    m_pMainWin->getpWinFileMgr()->getModelFileMap()[filePath]=!m_pMainWin->getpWinFileMgr()->getModelFileMap()[filePath];
    QMap<QString, bool>& fileList = m_pMainWin->getpWinFileMgr()->getModelFileMap();  // 获取 QMap 的引用
    QMap<QString, bool>::const_iterator it;
    for (it = fileList.cbegin(); it != fileList.cend(); ++it) {
        qDebug() << "Key:" << it.key() << ", Value:" << it.value();
    }

    m_pMainWin->getPWinVtkWidget()->UpdateInfo();
}

void FileManagerWidget::changeMeasuredFile(const QModelIndex &index){
    QStandardItem *item = model->itemFromIndex(index);
    QString filePath = item->data(Qt::UserRole).toString();
    m_pMainWin->getpWinFileMgr()->getMeasuredFileMap()[filePath]=!m_pMainWin->getpWinFileMgr()->getMeasuredFileMap()[filePath];
    QMap<QString, bool>& fileList = m_pMainWin->getpWinFileMgr()->getMeasuredFileMap();  // 获取 QMap 的引用
    QMap<QString, bool>::const_iterator it;
    for (it = fileList.cbegin(); it != fileList.cend(); ++it) {
        qDebug() << "Key:" << it.key() << ", Value:" << it.value();
    }

    m_pMainWin->getPWinVtkWidget()->UpdateInfo();
}

void FileManagerWidget::changeContentItem(const QModelIndex &index){
    // QStandardItem* childItem = model->itemFromIndex(index);
    // if (index.parent().isValid()&&isChildOf(childItem, contentItem)){
    //     m_pMainWin->m_EntityListMgr->getMarkList()[index.row()]=!m_pMainWin->m_EntityListMgr->getMarkList()[index.row()];//改变元素是否显示的标记
    //     int childCount = contentItem->rowCount();
    //     QVector<CEntity*> entityList = m_pMainWin->m_EntityListMgr->getEntityList();
    //     QVector<bool> marklist=m_pMainWin->m_EntityListMgr->getMarkList();
    //     for(int i=0;i<childCount;i++){
    //         QStandardItem* item = contentItem->child(i);
    //         bool isBtnOpen = item->data(Qt::UserRole+1).toBool();
    //         if(!isBtnOpen){
    //             marklist[i]=true;//不显示
    //         }
    //     }
    //     m_pMainWin->NotifySubscribe();
    // }
    QStandardItem *item = model->itemFromIndex(index);
    QString key = item->data(Qt::UserRole).toString();
    m_pMainWin->getpWinFileMgr()->getContentItemMap()[key]=!m_pMainWin->getpWinFileMgr()->getContentItemMap()[key];
    QMap<QString, bool>& contentList = m_pMainWin->getpWinFileMgr()->getContentItemMap();  // 获取 QMap 的引用
    QMap<QString, bool>::const_iterator it;
    for (it = contentList.cbegin(); it != contentList.cend(); ++it) {
        qDebug() << "Key:" << it.key() << ", Value:" << it.value();
    }

    m_pMainWin->getPWinVtkWidget()->UpdateInfo();
}

void FileManagerWidget::changeIdentifyItem(const QModelIndex &index){
    QStandardItem *item = model->itemFromIndex(index);
    QString key = item->data(Qt::UserRole).toString();
    m_pMainWin->getpWinFileMgr()->getIdentifyItemMap()[key]=!m_pMainWin->getpWinFileMgr()->getIdentifyItemMap()[key];
    QMap<QString, bool>& contentList = m_pMainWin->getpWinFileMgr()->getIdentifyItemMap();  // 获取 QMap 的引用
    QMap<QString, bool>::const_iterator it;
    for (it = contentList.cbegin(); it != contentList.cend(); ++it) {
        qDebug() << "Key:" << it.key() << ", Value:" << it.value();
    }

    m_pMainWin->getPWinVtkWidget()->UpdateInfo();
}

bool FileManagerWidget::isChildOf(QStandardItem* childItem, QStandardItem* parentItem) {
    if (!childItem || !parentItem){
        return false; // 检查指针有效性
    }
    // 遍历父项的所有直接子项
    for (int i = 0; i < parentItem->rowCount(); i++) {
        QStandardItem* item = parentItem->child(i);
        if (item == childItem) {
            return true; // 找到子项
        }
        // //如果当前子项还有子项，递归检查
        // if (isChildOf(childItem, item)) {
        //     return true;
        // }
    }
    return false; // 没有找到子项
}

void FileManagerWidget::UpdateInfo(){
    //删除contentItem中的所有子项
    // int childCount = contentItem->rowCount(); // 获取子项数量
    // for (int i=childCount-1;i>=0;i--) {
    //     contentItem->removeRow(i); // 删除子项
    // }

    // //显示构建的内容
    // QVector<bool> marklist=m_pMainWin->m_EntityListMgr->getMarkList();
    // QVector<CEntity*> entityList = m_pMainWin->m_EntityListMgr->getEntityList();
    // for(int i=0;i<entityList.size();i++){
    //     if(marklist[i]){
    //         createPresetClose(entityList[i]);
    //         qDebug()<<"关闭";
    //     }else{
    //         createPresetOpen(entityList[i]);
    //         qDebug()<<"打开";
    //     }
    // }
    createContentItem();
    createIdentifyItem();
}


//用于绘制图形元素到界面上//包含绘制项时需要的样式选项，如项的矩形区域、样式等//表示当前绘制的模型项的索引
void ButtonDelegate::paint(QPainter *painter,const QStyleOptionViewItem &option,const QModelIndex &index) const{
    QStyledItemDelegate::paint(painter,option,index);//调用基类的paint方法，执行默认绘制操作

    if (index.parent().isValid()) {//确保不是头节点
        bool isOpen=index.data(Qt::UserRole+1).toBool();//根据索引的数据决定给按钮添加哪种图标
        QStyleOptionButton btn;
        QRect itemRect  = option.rect;
        int buttonHeight = itemRect.height();
        int buttonWidth = buttonHeight + 10;
        int buttonX = itemRect.right() - buttonWidth - 10;//按钮水平位置的起始点
            //获取itemRect的y坐标,确保按钮在项的顶部对齐
        QRect buttonRect(buttonX, itemRect.top(), buttonWidth, buttonHeight);
        btn.rect = buttonRect;
        btn.icon=isOpen ? QIcon(":/component/eye/openeye.png") : QIcon(":/component/eye/closeeye.png");
        btn.iconSize = QSize(buttonWidth, buttonHeight);
        QApplication::style()->drawControl(QStyle::CE_PushButton, &btn, painter);//绘制按钮
    }
}

bool ButtonDelegate::editorEvent(QEvent *event, QAbstractItemModel *model, const QStyleOptionViewItem &option, const QModelIndex &index){
    if(event->type()==QEvent::MouseButtonRelease&&index.parent().isValid()){
        QMouseEvent *mouseEvent=static_cast<QMouseEvent*>(event);
        QRect itemRect = option.rect;
        int buttonHeight = itemRect.height();
        int buttonWidth = buttonHeight + 10;
        int buttonX = itemRect.right() - buttonWidth - 10;
        QRect buttonRect(buttonX, itemRect.top(), buttonWidth, buttonHeight);
        //buttonRect.moveRight(option.rect.right());//将按钮移动到右侧
        if(buttonRect.contains(mouseEvent->pos())){
            bool isOpen=index.data(Qt::UserRole+1).toBool();
            model->setData(index, !isOpen, Qt::UserRole+1);
            QString message = !isOpen ? "显示文件内容" : "不显示文件内容";
            qDebug() << message; //根据按钮状态输出不同的消息
            emit buttonClicked(index); // 发出信号
            return true;
        }
    }
    return QStyledItemDelegate::editorEvent(event, model, option, index);
}
