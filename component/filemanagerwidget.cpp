#include "filemanagerwidget.h"

#include <QMainWindow>
#include <QDebug>
#include <QIcon>

#include "mainwindow.h"

FileManagerWidget::FileManagerWidget(QWidget *parent)
    : QWidget{parent}
{
    m_pMainWin=(MainWindow*)parent;

    layout=new QVBoxLayout(this);

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

    layout->addWidget(filetree);
    layout->setContentsMargins(0, 0, 0, 0);//消除边距

    connect(filetree, &QTreeView::clicked, this, &FileManagerWidget::getItem);
    connect(delegate, &ButtonDelegate::buttonClicked, this, &FileManagerWidget::changeVtk);

    contextMenu = new QMenu(this);
    deleteAction = new QAction("删除文件", this);
    contextMenu->addAction(deleteAction);

    connect(deleteAction,&QAction::triggered,this,&FileManagerWidget::deleteFile);
}

void FileManagerWidget::openModelFile(QString fileName,QString filePath){
    QStandardItem *newFileItem = new QStandardItem(fileName);
    newFileItem->setData(filePath, Qt::UserRole);//在QTreeView的子节点中存储文件路径（Qt::UserRole用来存储用户第一个定义的数据：文件路径）
    newFileItem->setData(true, Qt::UserRole+1); //设置初始数据以便按钮绘制（Qt::UserRole+1用来存储用户第二个定义的数据：按钮状态）
    modelFile->appendRow(newFileItem);
}

void FileManagerWidget::openMeasuredFile(QString fileName,QString filePath){
    QStandardItem *newFileItem = new QStandardItem(fileName);
    newFileItem->setData(filePath, Qt::UserRole);//在QTreeView的子节点中存储文件路径
    newFileItem->setData(true, Qt::UserRole+1);
    measuredFile->appendRow(newFileItem);
}

void FileManagerWidget::createPresetOpen(CEntity *obj){
    QString str=obj->m_strCName+"  "+obj->m_strAutoName;
    QStandardItem *newObjItem = new QStandardItem(str);
    newObjItem->setData(true, Qt::UserRole+1);
    newObjItem->setData(QVariant::fromValue<CEntity*>(obj),Qt::UserRole+2);
    contentItem->appendRow(newObjItem);
}

void FileManagerWidget::createPresetClose(CEntity *obj){
    QString str=obj->m_strCName+"  "+obj->m_strAutoName;
    QStandardItem *newObjItem = new QStandardItem(str);
    newObjItem->setData(false, Qt::UserRole+1);
    newObjItem->setData(QVariant::fromValue<CEntity*>(obj),Qt::UserRole+2);
    contentItem->appendRow(newObjItem);
}

//显示右键菜单
void FileManagerWidget::contextMenuEvent(QContextMenuEvent *event){
    QModelIndex index=filetree->indexAt(event->pos());//用来获取在QTreeView中鼠标点击的位置
    if(index.isValid()){
        qDebug()<<"进入";
        contextMenu->exec(event->globalPos());//用来显示右键菜单
    }
}

//从项目中删除文件
void FileManagerWidget::deleteFile(){
    QModelIndex index=filetree->currentIndex();
    if(index.isValid()){
        QStandardItem *item = model->itemFromIndex(index);//获取被点击的项
        QString filePath = item->data(Qt::UserRole).toString();//从子节点中获取文件路径
        QFileInfo fileInfo(filePath);
        //注释部分是彻底从电脑上删除文件，而不是从项目中把文件移除
        if(fileInfo.isFile()){//判断点击的是否为文件
            //QFile file(filePath);
            //if(file.remove()){                //行号         //父索引
            bool removed=model->removeRow(index.row(),index.parent());
            if(removed){
                qDebug()<<"文件被删除";
            }else{
                qDebug()<<"文件没有被删除";
            }
            //}else{
            //qDebug()<<"文件没有被删除";
            //}
        }else{
            qDebug()<<"选择的不是文件";
        }
    }
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

void FileManagerWidget::changeVtk(const QModelIndex &index){
    QStandardItem* childItem = model->itemFromIndex(index);
    if (index.parent().isValid()&&isChildOf(childItem, contentItem)){
        m_pMainWin->m_EntityListMgr->getMarkList()[index.row()]=!m_pMainWin->m_EntityListMgr->getMarkList()[index.row()];//改变元素是否显示的标记
        int childCount = contentItem->rowCount();
        QVector<CEntity*> entityList = m_pMainWin->m_EntityListMgr->getEntityList();
        QVector<bool> marklist=m_pMainWin->m_EntityListMgr->getMarkList();
        for(int i=0;i<childCount;i++){
            QStandardItem* item = contentItem->child(i);
            bool isBtnOpen = item->data(Qt::UserRole+1).toBool();
            if(!isBtnOpen){
                marklist[i]=true;//不显示
            }
        }
        m_pMainWin->NotifySubscribe();
    }
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
        // 如果当前子项还有子项，递归检查
        // if (isChildOf(childItem, item)) {
        //     return true;
        // }
    }
    return false; // 没有找到子项
}

void FileManagerWidget::UpdateInfo(){
    //删除contentItem中的所有子项
    int childCount = contentItem->rowCount(); // 获取子项数量
    for (int i=childCount-1;i>=0;i--) {
        contentItem->removeRow(i); // 删除子项
    }

    //显示构建的内容
    QVector<bool> marklist=m_pMainWin->m_EntityListMgr->getMarkList();
    QVector<CEntity*> entityList = m_pMainWin->m_EntityListMgr->getEntityList();
    for(int i=0;i<entityList.size();i++){
        if(marklist[i]){
            createPresetClose(entityList[i]);
            qDebug()<<"关闭";
        }else{
            createPresetOpen(entityList[i]);
            qDebug()<<"打开";
        }
    }
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
