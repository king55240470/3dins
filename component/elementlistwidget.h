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
#include <QDebug>
#include <QContextMenuEvent>
#include<QToolBar>
#include <QVector>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QAction>
#include <QStateMachine>
#include <QState>
#include <QFinalState>
#include <QSignalTransition>
#include <QPropertyAnimation>
#include <QTimer>
#include <QQueue>
#include "geometry/centity.h"
#include "toolwidget.h"
#include "mainwindow.h"
#include "DataWidget.h"
#include "toolwidget.h"
#include"vtkwindow/vtkwidget.h"
#include"constructor/distanceconstructor.h"
#include"constructor/planeconstructor.h"
#include"constructor/angleconstructor.h"
#include"vtkwindow/vtkpresetwidget.h"

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
    QList<QTreeWidgetItem*> getSelectedItems();
    QVector<CObject*> getEleobjlist();
    void changeName();
    void setAutoName();
    void starttime();
    void selectall();
    void showDialog();
    void selectcommonitem();
    void onItemClicked();
    void setTolerance();
    void BtnClicked();
    void ShowParent(CObject*obj);
    void showInfotext();
    void closeInfotext();
    void mousePressEvent(QMouseEvent *event) override;
    void setupStateMachine();
    void continueUpdate();
    void onAddElement(pcl::PointCloud<pcl::PointXYZRGB>::Ptr could);
    void CompareCloud();
    void updateDistance();
    void startupdateData(pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree, QVector<CEntity*>distancelist);
    void isAdd();
    void createrule();
    CPlane* UpdateFindPlane(CObject*FindPoint,pcl::PointXYZRGB searchPoint,pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree);

protected:
    void mouseDoubleClickEvent(QMouseEvent *event) override;
    void keyPressEvent(QKeyEvent *event) override;
    void keyReleaseEvent(QKeyEvent *event) override;
private:

    QTreeWidget *treeWidgetNames;
    QTreeWidget *treeWidgetInfo;
    QLineEdit *xLineEdit;
    QLineEdit *yLineEdit;
    QLineEdit *sizeLineEdit;
    int nextEllipseIndex=0;
    QToolBar * toolBar;
    CEntity * centity;
    MainWindow *m_pMainWin=nullptr;
    static int pcscount;
    QVector<CObject*>eleobjlist;
    //std::unordered_map<QTreeWidgetItem*, size_t> itemToIndexMap;
    bool ctrlPressed = false;
    QLineEdit* name;
    QLineEdit* up;
    QLineEdit* down;
    QPushButton *updownBtn;
    QPushButton *startButton;
    QPushButton *pauseButton;
    QPushButton *terminateButton;
    QPushButton *continueButton;
    QDialog *dialog;
    QStateMachine *stateMachine;
    QState *stoppedState;
    QState *runningState;
    QState *pausedState;
    QState *continueState;
    int Treelistsize=0;
    int currentIndex;
    int distancelistIndex;
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    QVector<CEntity*>disAndanglelist;
    QTimer* timer=nullptr;
    QQueue<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>pointCouldlists;
    bool isProcessing=false;
    QVector<CEntity*>list;
signals:
    void itemSelected(int index);
};

#endif // ELEMENTLISTWIDGET_H
