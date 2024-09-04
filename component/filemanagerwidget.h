#ifndef FILEMANAGERWIDGET_H
#define FILEMANAGERWIDGET_H

#include <QWidget>
#include <QFileSystemModel>
#include <QTreeView>
#include <QStandardItemModel>
#include <QVBoxLayout>
#include <QFile>
#include <QMessageBox>

class VtkWidget;

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
public:
    void openModelFile(QString,QString);
    void getFileContent(const QString &);
private slots:
    void clickFile(const QModelIndex &);
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

#endif // FILEMANAGERWIDGET_H
