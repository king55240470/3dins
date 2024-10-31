#ifndef CUSTOMINTERACTORSTYLE_H
#define CUSTOMINTERACTORSTYLE_H

#include "vtkwindow/vtkwidget.h"
#include "vtkwindow/clickhighlightstyle.h"
#include <vtkInteractionStyleModule.h>

class CustomInteractorStyle : public vtkInteractorStyleTrackballCamera
{
public:
    static CustomInteractorStyle* New();
    vtkTypeMacro(CustomInteractorStyle, vtkInteractorStyleTrackballCamera);
    CustomInteractorStyle();

    // 设置渲染器
    void SetRenderer(vtkRenderer* renderer) {
        this->renderer = renderer;
    }

    // 管理鼠标事件
    // virtual void OnLeftButtonDown() override;
    // virtual void OnLeftButtonUp() override;
    virtual void OnRightButtonDown() override;
    virtual void OnRightButtonUp() override;
    virtual void OnMouseMove() override;

private:
    vtkSmartPointer<vtkRenderer> renderer; // 渲染器成员
    // MouseInteractorHighlightActor* highlightstyle = nullptr; // 高亮样式成员

    bool RightButtonPressed; // 用于区分单击和拖拽
    int RightButtonDownPosition[2]; // 用于跟踪右键按下的位置

};

#endif // CUSTOMINTERACTORSTYLE_H
