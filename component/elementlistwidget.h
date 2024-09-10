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
//#include <unordered_map>
#include "geometry/globes.h"
class ElementListWidget : public QWidget
{
    Q_OBJECT
public:
    explicit ElementListWidget(QWidget *parent = nullptr);
    void onCreateEllipse();
    void CreateEllipse(CObject*);
    void onDeleteEllipse();
    int getNextId();
    void onCustomContextMenuRequested(const QPoint &pos);
    void deal_actionNew_triggered();
    void judgetype(CEntity*);
    void removeall();
    void updateInsertIndicatorPosition();
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
    ENTITY_TYPE m_EntityType;

    //std::unordered_map<QTreeWidgetItem*, size_t> itemToIndexMap;
};

#endif // ELEMENTLISTWIDGET_H
