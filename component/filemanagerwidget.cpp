#include "filemanagerwidget.h"

#include <QMainWindow>
#include <QDebug>
#include <QIcon>

#include "component/vtkwidget.h"

FileManagerWidget::FileManagerWidget(QWidget *parent)
    : QWidget{parent}
{
    layout=new QVBoxLayout(this);

    filetree=new QTreeView(this);

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

    //if(!rootItem){
    connect(filetree, &QTreeView::clicked, this, &FileManagerWidget::clickFile);
    //}

    contextMenu = new QMenu(this);
    deleteAction = new QAction("删除文件", this);
    contextMenu->addAction(deleteAction);

    connect(deleteAction,&QAction::triggered,this,&FileManagerWidget::deleteFile);
}

void FileManagerWidget::openModelFile(QString fileName,QString filePath){
    QStandardItem *newFileItem = new QStandardItem(fileName);
    newFileItem->setData(filePath, Qt::UserRole);//在QTreeView的子节点中存储文件路径（Qt::UserRole用来存储用户定义的数据）
    newFileItem->setData(true, Qt::UserRole); //设置初始数据以便按钮绘制
    modelFile->appendRow(newFileItem);
}

void FileManagerWidget::openMeasuredFile(QString fileName,QString filePath){
    QStandardItem *newFileItem = new QStandardItem(fileName);
    newFileItem->setData(filePath, Qt::UserRole);//在QTreeView的子节点中存储文件路径
    measuredFile->appendRow(newFileItem);
}

//显示右键菜单
void FileManagerWidget::contextMenuEvent(QContextMenuEvent *event){
    QModelIndex index=filetree->indexAt(event->pos());//用来获取在QTreeView中鼠标点击的位置
    if(index.isValid()){
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
            //if(file.remove()){
                model->removeRow(index.row(),index.parent());
                qDebug()<<"文件被删除";
            //}else{
                //qDebug()<<"文件没有被删除";
            //}
        }else{
            qDebug()<<"选择的不是文件";
        }
    }
}

//点击文件名获得文件路径&按钮状态
void FileManagerWidget::clickFile(const QModelIndex &index){
    QStandardItem *item = model->itemFromIndex(index);//获取被点击的项
    QString filePath = item->data(Qt::UserRole).toString();//从子节点中获取文件路径
    bool isBtnOpen = item->data(Qt::UserRole).toBool();//从子节点中获取按钮状态
    QString status = isBtnOpen ? "按钮状态: 显示文件内容" : "按钮状态: 不显示文件内容";
    qDebug() << status;
    //getFileContent(filePath);
}

/*void FileManagerWidget::getFileContent(const QString &path){
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) {
        qDebug() << "无法打开文件:" << file.errorString();
        return; // 不进行后续操作
    }
    QTextStream in(&file);
    QString content = in.readAll();
    file.close();
    //if (pWinVtkWidget) {
        //delete pWinVtkWidget;//如果已经创建了pWinVtkWidget，先删除
    //}
    pWinVtkWidget=new VtkWidget();
    //pWinVtkWidget->setContent(content);
    //file.close();
}*/
