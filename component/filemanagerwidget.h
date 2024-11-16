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

class MainWindow;
class ButtonDelegate;
class FileManagerWidget : public QWidget
{
    Q_OBJECT
public:
    explicit FileManagerWidget(QWidget *parent = nullptr);
private:
    QVBoxLayout *layout;
    QPushButton *compareBtn;
    QTreeView *filetree;
    QStandardItemModel *model;
    QStandardItem *rootItem;
    QStandardItem *modelFile;
    QStandardItem *measuredFile;
    QStandardItem *contentItem;
    QStandardItem *identifyItem;
    ButtonDelegate *delegate;
    QMenu *contextMenu;//右键菜单
    QAction *deleteAction;
    MainWindow *m_pMainWin;

    QModelIndex selectedIndex;//保存右键删除时选中的索引
    QStandardItem *selectedItem;//保存右键删除时选中的子项
public:
    void openModelFile(QString,QString);
    void openMeasuredFile(QString,QString);
    void createContentItem();
    void createIdentifyItem();
    // void createPresetOpen(CEntity*);
    // void createPresetClose(CEntity*);
    void showContextMenu(const QPoint &);
    void UpdateInfo();
    bool isChildOf(QStandardItem*, QStandardItem*);
private slots:
    void getItem(const QModelIndex &);
    void changePlay(const QModelIndex &);
    void changeModelFile(const QModelIndex &);
    void changeMeasuredFile(const QModelIndex &);
    void changeContentItem(const QModelIndex &);
    void changeIdentifyItem(const QModelIndex &);
    void deleteFile();
signals:
};

//自定义代理类，用于绘制按钮
class ButtonDelegate : public QStyledItemDelegate {
    Q_OBJECT
public:
    ButtonDelegate(QObject *parent = nullptr) : QStyledItemDelegate(parent) {}
    void paint(QPainter *painter,const QStyleOptionViewItem &option,const QModelIndex &index) const override;
    bool editorEvent(QEvent *event, QAbstractItemModel *model, const QStyleOptionViewItem &option, const QModelIndex &index) override;
signals:
    void buttonClicked(const QModelIndex &index); // 自定义信号，包含项的索引
};

#endif // FILEMANAGERWIDGET_H
