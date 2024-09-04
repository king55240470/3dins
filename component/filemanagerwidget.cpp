#include "filemanagerwidget.h"

#include <QMainWindow>
#include <QDebug>

#include "component/vtkwidget.h"

FileManagerWidget::FileManagerWidget(QWidget *parent)
    : QWidget{parent}
{
    layout=new QVBoxLayout(this);

    filetree=new QTreeView(this);

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
}

void FileManagerWidget::openModelFile(QString fileName,QString filePath){
    QStandardItem *newFileItem = new QStandardItem(fileName);
    newFileItem->setData(filePath, Qt::UserRole);//在QTreeView的子节点中存储文件路径
    modelFile->appendRow(newFileItem);
}

void FileManagerWidget::clickFile(const QModelIndex &index){
    QStandardItem *item = model->itemFromIndex(index);//获取被点击的项
    QString filePath = item->data(Qt::UserRole).toString();//从子节点中获取文件路径
    getFileContent(filePath);
}

void FileManagerWidget::getFileContent(const QString &path){
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) {
        qDebug() << "无法打开文件:" << file.errorString();
        return; // 不进行后续操作
    }
    QTextStream in(&file);
    QString content = in.readAll();
    file.close();
    /*if (pWinVtkWidget) {
        delete pWinVtkWidget;//如果已经创建了pWinVtkWidget，先删除
    }*/
    pWinVtkWidget=new VtkWidget();
    //pWinVtkWidget->setContent(content);
    //file.close();
}
