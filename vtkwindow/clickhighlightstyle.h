#ifndef CLICKSELECT_H
#define CLICKSELECT_H

#include "geometry/globes.h"

#include <QVector>
#include <vtkSmartPointer.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkGenericRenderWindowInteractor.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkPropPicker.h>
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
    // 静态方法，用于创建类的实例4
    static MouseInteractorHighlightActor* New();
    // 宏，用于设置类的类型和名称
    vtkTypeMacro(MouseInteractorHighlightActor, vtkInteractorStyleTrackballCamera);

    // 重写左键按下事件，点击高亮显
    virtual void OnLeftButtonDown() override;

    // 重写右键按下事件，取消高亮
    virtual void OnRightButtonDown() override;

    // 绑定渲染器
    void SetRenderer(vtkRenderer* renderer);

    // 高亮显示指定的actor
    void HighlightActor(vtkActor* actor);

    // 恢复指定的actor的属性到之前的状态
    void ResetActor(vtkActor* actor);

    // 获取PickedActors
    QVector<std::pair<vtkSmartPointer<vtkActor>,vtkSmartPointer<vtkProperty>>>& getPickedActors();

private:
    // 存储所有选中的actor和原始属性
    QVector<std::pair<vtkSmartPointer<vtkActor>,vtkSmartPointer<vtkProperty>>> pickedActors;

    // 存储所有actor
    QVector<vtkActor*> actors;

    // 渲染器和交互器
    vtkRenderer* renderer = nullptr;
    vtkGenericRenderWindowInteractor* interactor = nullptr;

};


#endif // CLICKSELECT_H
