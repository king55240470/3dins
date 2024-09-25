#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSplitter>
#include <QMenuBar>
#include <QStatusbar>
#include <QPushButton>
#include "geometry/globes.h"
#include "geometry/centity.h"
//#include "component/vtkwidget.h"
#include "manager/centitymgr.h"
#include "manager/cobjectmgr.h"
#include "manager/cpcsmgr.h"

class DataWidget;
class ElementListWidget;
class FileManagerWidget;
class ToolWidget;
class VtkPresetWidget;
class VtkWidget;
class ReportWidget;
class LogWidget;
class PresetElemWidget;

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

    QMenuBar * bar;

    //状态栏
    QStatusBar *stbar;
    QPushButton* switchRefCsBtn;
    QPushButton* switchCsBtn;

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
    //预置
    void OnPresetPoint(CPosition pt);
    void OnPresetLine(CPosition ptStart, CPosition ptEnd);
    void OnPresetCircle(CPosition pt, double diameter);
    void OnPresetPlane(CPosition posCenter, QVector4D normal, QVector4D direction, double length, double width);
    void OnPresetSphere(CPosition posCenter, double diametre);
    void OnPresetCylinder(CPosition pos, QVector4D vec, double height, double diametre);
    void OnPresetCone(CPosition posCenter, QVector4D axis, double partH, double fullH, double angle);
    CObjectMgr *getObjectListMgr();
    CEntity* CreateEntity(int nType);
    void NotifySubscribe();
    void loadManager();

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

private:
    // RELY_ON_CS_TYPE m_nRelyOnWhichCs;
    CPcs* m_nRelyOnWhichCs; // 状态栏的参考坐标系
};
#endif // MAINWINDOW_H
