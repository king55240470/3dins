#ifndef CLICKSELECT_H
#define CLICKSELECT_H

#include "mainwindow.h"
#include "geometry/globes.h"
#include "manager/chosencentitymgr.h"

#include <QVector>
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
    // virtual void KeyPressEvent(char key) override;

    void SetRenderer(vtkRenderer* renderer);     // 绑定渲染器
    void SetUpChosenListMgr(ChosenCEntityMgr* chosenListMgr); // 初始化chosenlistmgr
    void SetUpMainWin(MainWindow* mainwindow){ // 初始化m_pMainWin
        m_pMainWin = mainwindow;
    }

    // 生成一个用于高亮的顶点
    vtkActor* CreatHighLightPoint(double pos[3]);

    // 高亮显示指定的actor
    void HighlightActor(vtkActor* actor);

    // 恢复指定的actor的属性到之前的状态
    void ResetActor(vtkActor* actor);

    // 恢复被点击选中的状态
    void BackChoosen(vtkActor* actor);

    // 获取PickedActors
    QVector<std::pair<vtkSmartPointer<vtkActor>,vtkSmartPointer<vtkProperty>>>& getPickedActors();

private:
    MainWindow* m_pMainWin = nullptr; // mainwindow指针
    ChosenCEntityMgr* chosenlistmgr; // 选中元素列表的管理器成员

    // 存储所有选中的actor和原始属性
    QVector<std::pair<vtkSmartPointer<vtkActor>,vtkSmartPointer<vtkProperty>>> pickedActors;

    // 存储所有用于高亮的顶点
    QVector<vtkActor*> point_actors;

    // 渲染器和交互器
    vtkRenderer* renderer = nullptr;
    vtkGenericRenderWindowInteractor* interactor = nullptr;

signals:
    void MouseClicked(double pos[3]); // 定义信号，向qt窗口发送选中的坐标

};


#endif // CLICKSELECT_H
