#ifndef ELEMENTLISTWIDGET_H
#define ELEMENTLISTWIDGET_H

#include <QWidget>
#include <QLabel>
#include <QPushButton>
#include <QMenu>
#include <QContextMenuEvent>
#include <QVBoxLayout>
#include <QMessageBox>
#include <QLineEdit>
#include <QHBoxLayout>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QApplication>
#include <set>
#include <QDebug>
#include <QContextMenuEvent>
#include<QToolBar>
class ElementListWidget : public QWidget
{
    Q_OBJECT
public:
    explicit ElementListWidget(QWidget *parent = nullptr);
private slots:
    void onCreateEllipse();
    void onDeleteEllipse();
    int getNextId();
    void onCustomContextMenuRequested(const QPoint &pos);
    void deal_actionNew_triggered();
private:
    QTreeWidget *treeWidgetNames;
    QTreeWidget *treeWidgetInfo;
    QLineEdit *xLineEdit;
    QLineEdit *yLineEdit;
    QLineEdit *sizeLineEdit;
    int nextEllipseIndex=0;
    std::set<int> deletedIds;
    QToolBar * toolBar;
    QVector<QString> vct ={"11","22"};//假设，作为例子
signals:
};

#endif // ELEMENTLISTWIDGET_H
