#ifndef ELEMENTLISTWIDGET_H
#define ELEMENTLISTWIDGET_H
#include "geometry/centity.h"
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
#include <QVector>
//#include "manager/centitymgr.h"
//#include <unordered_map>
//#include "geometry/globes.h"
#include "mainwindow.h"
class ElementListWidget : public QWidget
{
    Q_OBJECT
public:
    explicit ElementListWidget(QWidget *parent = nullptr);
    void CreateEllipse(CObject*);
    void onDeleteEllipse();
    int getNextId();
    void onCustomContextMenuRequested(const QPoint &pos);
    void deal_actionNew_triggered();
    void judgetype(CEntity*);
    void removeall();
    void updateInsertIndicatorPosition();
    void upadteelementlist();

private:
    QTreeWidget *treeWidgetNames;
    QTreeWidget *treeWidgetInfo;
    QLineEdit *xLineEdit;
    QLineEdit *yLineEdit;
    QLineEdit *sizeLineEdit;
    int nextEllipseIndex=0;
    std::set<int> deletedIds;
    QToolBar * toolBar;
    CEntity * centity;
    MainWindow *m_pMainWin=nullptr;
    //std::unordered_map<QTreeWidgetItem*, size_t> itemToIndexMap;
signals:
    void itemSelected(int itemName);
};

#endif // ELEMENTLISTWIDGET_H
