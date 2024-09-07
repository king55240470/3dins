#ifndef FILEMANAGERWIDGET_H
#define FILEMANAGERWIDGET_H

#include <QWidget>
#include <QFileSystemModel>
#include <QTreeView>
#include <QStandardItemModel>
#include <QVBoxLayout>
#include <QFile>
#include <QMessageBox>
#include <QPushButton>
#include <QMouseEvent>
#include <QMenu>
#include <QAction>
#include <QIcon>

#include <QStyledItemDelegate>
#include <QRect>
#include <QPainter>
#include <QApplication>

class VtkWidget;
class ButtonDelegate;
class FileManagerWidget : public QWidget
{
    Q_OBJECT
public:
    explicit FileManagerWidget(QWidget *parent = nullptr);
private:
    QVBoxLayout *layout;
    QTreeView *filetree;
    QStandardItemModel *model;
    QStandardItem *rootItem;
    QStandardItem *modelFile;
    QStandardItem *measuredFile;
    QStandardItem *contentItem;
    QStandardItem *identifyItem;
    VtkWidget *pWinVtkWidget;
    ButtonDelegate *delegate;
    QMenu *contextMenu;//右键菜单
    QAction *deleteAction;
public:
    void openModelFile(QString,QString);
    void openMeasuredFile(QString,QString);
    //void getFileContent(const QString &);
    void contextMenuEvent(QContextMenuEvent *event);
private slots:
    void clickFile(const QModelIndex &);
    void deleteFile();
private:
    /*QString getPath(QStandardItem *item){
        QString path;
        while (item) {
            path.prepend(item->text() + "/");
            item = item->parent();
        }
        return path.left(path.length() - 1);
    }*/
signals:
};

//自定义代理类，用于绘制按钮
class ButtonDelegate : public QStyledItemDelegate {
    Q_OBJECT
public:
    ButtonDelegate(QObject *parent = nullptr) : QStyledItemDelegate(parent) {}

        // 用于绘制图形元素到界面上 //包含绘制项时需要的样式选项，如项的矩形区域、样式等 //表示当前绘制的模型项的索引
    void paint(QPainter *painter,const QStyleOptionViewItem &option,const QModelIndex &index) const override{
        QStyledItemDelegate::paint(painter,option,index);//调用基类的paint方法，执行默认绘制操作

        /*if (index.column() == 1){
        QRect buttonRect=option.rect;//用于创建一个矩形区域
        buttonRect.setLeft(buttonRect.right() - 50);//使按钮在子节点左侧
        buttonRect.setWidth(50);
        buttonRect.setHeight(20);
        painter->drawRect(buttonRect);//绘制按钮边框
        QIcon iconOpen=QIcon(":/component/eye/openeye.png");
        iconOpen.paint(painter,buttonRect,Qt::AlignCenter);
        }*/

        if (index.parent().isValid()) {//确保不是头节点
            bool isOpen=index.data(Qt::UserRole+1).toBool();//根据索引的数据决定是否绘制按钮
            QStyleOptionButton btn;
            btn.rect=option.rect.adjusted(280,0,-5,0);
            btn.icon=isOpen ? QIcon(":/component/eye/openeye.png") : QIcon(":/component/eye/closeeye.png");
            btn.iconSize = QSize(20,20);
            QApplication::style()->drawControl(QStyle::CE_PushButton, &btn, painter);//绘制按钮
        }
    }

    bool editorEvent(QEvent *event, QAbstractItemModel *model, const QStyleOptionViewItem &option, const QModelIndex &index) override {
        if(event->type()==QEvent::MouseButtonRelease&&index.parent().isValid()){
            QMouseEvent *mouseEvent=static_cast<QMouseEvent*>(event);
            QRect buttonRect=option.rect.adjusted(280,0,-5,0);
            //buttonRect.moveRight(option.rect.right());//将按钮移动到右侧
            if(buttonRect.contains(mouseEvent->pos())){
                bool isOpen=index.data(Qt::UserRole+1).toBool();
                model->setData(index, !isOpen, Qt::UserRole+1);
                QString message = !isOpen ? "显示文件内容" : "不显示文件内容";
                qDebug() << message; //根据按钮状态输出不同的消息
                return true;
            }
        }
        return QStyledItemDelegate::editorEvent(event, model, option, index);
    }

private:
    ButtonDelegate *delegate;
};

#endif // FILEMANAGERWIDGET_H
