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
#include <QThread>
#include <QProgressBar>
#include "geometry/centity.h"
#include "toolwidget.h"
#include "mainwindow.h"
#include "DataWidget.h"
#include "toolwidget.h"
#include"vtkwindow/vtkwidget.h"
#include"constructor/distanceconstructor.h"
#include"constructor/planeconstructor.h"
#include"constructor/lineconstructor.h"
#include"constructor/angleconstructor.h"
#include"vtkwindow/vtkpresetwidget.h"
#include"pointfitting/fittingline.h"
#include"component/worker.h"
#include"component/progressdialogs.h"

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
    void beginStartButton();
    void startprocess();
    void onAddElement(pcl::PointCloud<pcl::PointXYZRGB>::Ptr could,QString type);
    void CompareCloud();
    void updateDistance();//开启更新
    void startupdateData(pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree, QVector<CEntity*>distancelist);//timer关联函数
    void UpdateDisNowFun(QVector<CEntity*>distancelist);//更新距离元素
    void isAdd();
    void createrule();
    bool getIsProcess();
    void setModelIndex(int index);
    QVector<CPointCloud*> getisComparsionCloud();
    QTreeWidget *getTreeWidgetNames();
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
    bool ctrlPressed = false;
    //更改别名
    QLineEdit* name;
    //距离元素设置上下公差
    QLineEdit* up;
    QLineEdit* down;
    //状态机启动按钮
    QPushButton *updownBtn;
    QPushButton *startButton;
    QPushButton *pauseButton;
    QPushButton *terminateButton;
    QPushButton *continueButton;

    QDialog *dialog;
    //状态机状态
    QStateMachine *stateMachine;
    QState *stoppedState;
    QState *runningState;
    QState *pausedState;
    QState *continueState;

    int Treelistsize=0;
    int currentIndex;//被测量主元素parent索引
    int distancelistIndex;//被测量主元素队列索引
    int modelIndex;//模型点云的索引，用于删除

    bool foundmodel;//检查是否有模型点云

    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    QVector<CEntity*>disAndanglelist;
    QTimer* timer=nullptr;
    QQueue<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>pointCouldlists;
    QQueue<QString>filetypelists;
    bool isProcessing=false;
    QVector<CEntity*>list;
    //创建线程用来进行保存文件工作
    QThread *workerThread;
    Worker* worker;
    QProgressBar *progressBar;
    ProgressDialogs *progressDialog;
    QVector<CPointCloud*>isComparsionCloudList;
signals:
    void itemSelected(int index);
public slots:
    void updateProgress(int value);
};

#endif // ELEMENTLISTWIDGET_H
