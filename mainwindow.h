#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSplitter>
#include <QMenuBar>
#include <QStatusbar>
#include <QPushButton>
#include "geometry/globes.h"
#include "geometry/centity.h"
#include "manager/centitymgr.h"
#include "manager/cobjectmgr.h"
#include "manager/cpcsmgr.h"
#include "manager/chosencentitymgr.h"
#include "manager/pointcloudlistmgr.h"
#include <QMap>//从actor映射到centity

class DataWidget;
class ElementListWidget;
class FileManagerWidget;
class ToolWidget;
class VtkPresetWidget;
class VtkWidget;
class ReportWidget;
class LogWidget;
class PresetElemWidget;
class FileMgr;
class setDataWidget;


class MainWindow : public QMainWindow
{
    Q_OBJECT
private:
    QSplitter *spMainWindow;
    QSplitter *spLeft;
    QSplitter *spMiddleup;
    QSplitter *spMiddledown;
    QSplitter *spRightup;
    QSplitter *spRightdown;

    DataWidget *pWinDataWidget;
    ElementListWidget *pWinElementListWidget;
    FileManagerWidget *pWinFileManagerWidget;
    ToolWidget *pWinToolWidget;
    VtkPresetWidget *pWinVtkPresetWidget;
    VtkWidget *pWinVtkWidget;
    ReportWidget *pWinReportWidget;
    LogWidget *pWinLogWidget;
    PresetElemWidget *pWinPresetElemWidget;

    FileMgr *pWinFileMgr;

    setDataWidget *pWinSetDataWidget;

    QMenuBar * bar;

    //状态栏
    QStatusBar *stbar;
    QPushButton* switchRefCsBtn;
    QPushButton* switchCsBtn;

    QMap<vtkSmartPointer<vtkActor>, CEntity*> actorToEntityMap;

protected:
    bool eventFilter(QObject* watched, QEvent* event) override;

    //私有的槽函数
    /*private slots:
    void open_clicked();
    void save_clicked();*/
private:
    void openFile();
    void saveFile();

public:
    CPcsMgr* m_pcsListMgr;
    CObjectMgr* m_ObjectListMgr;
    CEntityMgr* m_EntityListMgr;
    ChosenCEntityMgr* m_ChosenListMgr;
    PointCloudListMgr *m_CloudListMgr;

    //预置
    void OnPresetPoint(CPosition pt);
    void OnPresetLine(CPosition ptStart, CPosition ptEnd);
    void OnPresetCircle(CPosition pt, double diameter);
    void OnPresetPlane(CPosition posCenter, QVector4D normal, QVector4D direction, double length, double width);
    void OnPresetSphere(CPosition posCenter, double diametre);
    void OnPresetCylinder(CPosition pos, QVector4D vec, double height, double diametre);
    void OnPresetCone(CPosition posCenter, QVector4D axis, double partH, double fullH, double angle);
    void OnPresetCuboid(CPosition posCenter,double length,double width,double height,double angleX,double angleY,double angleZ);

    CObjectMgr *getObjectListMgr();
    CEntityMgr*getEntityListMgr();
    CEntity* CreateEntity(int nType);
    void NotifySubscribe();
    void loadManager();
    ElementListWidget *getPWinElementListWidget();
    VtkWidget * getPWinVtkWidget();
    DataWidget *getPWinDataWidget();
    ToolWidget *getPWinToolWidget();
    //slots
    void on2dCoordOriginAuto(); // 创建坐标系
    void on2dCoordSave();

    // 切换图形的观察视角
    void onTopViewClicked();
    void onRightViewClicked();
    void onFrontViewClicked();
    void onIsometricViewClicked();

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void LoadWidgets();
    void RestoreWidgets();
    void setupUi();
    void showPresetElemWidget(int);
    FileMgr *getpWinFileMgr();
    ChosenCEntityMgr *getChosenListMgr(); // 获取窗口选中点的管理器成员
    PointCloudListMgr *getPointCloudListMgr(); // 获取打开的点云列表
    setDataWidget *getPWinSetDataWidget();
    void LoadSetDataWidget();

public:
    RELY_ON_CS_TYPE m_nRelyOnWhichCs;

    //坐标系摆正
    double AxesRotateX(); // X-Y平面, 线段与X轴正方向之间的夹角
    void on2dCoordSetRightX(); // 摆正X坐标系-+
    void on2dCoordSetRightY(); // 摆正Y坐标系
public:
    QMap<vtkSmartPointer<vtkActor>, CEntity*>& getactorToEntityMap(); // 管理所有centity对象生成的actor
};
#endif // MAINWINDOW_H
