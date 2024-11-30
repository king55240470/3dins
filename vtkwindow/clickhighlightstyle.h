#ifndef CLICKSELECT_H
#define CLICKSELECT_H

#include "mainwindow.h"
#include "geometry/globes.h"
#include "manager/chosencentitymgr.h"
#include "manager/filemgr.h"

#include <QVector>
#include <QDebug>
#include <QObject>
#include <vtkSmartPointer.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkGenericRenderWindowInteractor.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkPropPicker.h>
#include <vtkCellPicker.h>
#include <vtkPointSource.h>
#include <vtkSphereSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkMath.h>
#include <vtkProperty.h>
#include <vtkCallbackCommand.h>
#include <vtkCommand.h>

// 定义一个继承自vtkInteractorStyleTrackballCamera的类，用于处理鼠标交互
class MouseInteractorHighlightActor : public vtkInteractorStyleTrackballCamera
{
public:
    // 静态方法，用于创建类的实例
    static MouseInteractorHighlightActor* New();
    // 宏，用于设置类的类型和名称
    vtkTypeMacro(MouseInteractorHighlightActor, vtkInteractorStyleTrackballCamera);
    MouseInteractorHighlightActor(vtkInteractorStyleTrackballCamera* parent = nullptr);

    virtual void OnLeftButtonDown() override; // 重写左键按下事件，点击高亮
    virtual void OnRightButtonDown() override; // 重写右键按下事件，取消高亮

    void SetRenderer(vtkRenderer* renderer);     // 绑定渲染器
    void SetUpMainWin(MainWindow* mainwindow){ // 初始化m_pMainWin
        m_pMainWin = mainwindow;
    }

    vtkActor* CreatHighLightPoint(double pos[3]); // 生成一个用于高亮的顶点
    void DeleteHighLightPoint(); // 删除所有临时高亮的顶点

    // 高亮显示指定的actor
    void HighlightActor(vtkActor* actor);

    // 恢复指定的actor的属性到之前的状态
    void ResetActor(vtkActor* actor);

    // 获取PickedActors
    QVector<std::pair<vtkSmartPointer<vtkActor>,vtkSmartPointer<vtkProperty>>>& getPickedActors();
    // 获取point_actors
    QVector<vtkActor*> getpoint_actors();

    void onCancelSelect(); // 取消选中

private:
    MainWindow* m_pMainWin = nullptr; // mainwindow指针

    // 存储所有选中的actor和原始属性
    QVector<std::pair<vtkSmartPointer<vtkActor>,vtkSmartPointer<vtkProperty>>> pickedActors;

    // 存储所有用于高亮的顶点
    QVector<vtkActor*> point_actors;

    // 渲染器和交互器
    vtkRenderer* renderer = nullptr;

    int lastRightClickPos[2]; // 记录上次右键点击位置
    vtkTimeStamp lastRightClickTime; // 时间戳，记录上次右键发生的 时间
    int DOUBLE_CLICK_INTERVAL = 300; // 双击间隔时间，单位毫秒

    QMenu* vtkMenu; // 右键双击菜单
signals:

};


#endif // CLICKSELECT_H
