#include "custominteractorstyle.h"

// 实现New()方法的宏
vtkStandardNewMacro(CustomInteractorStyle);

CustomInteractorStyle::CustomInteractorStyle() {}


void CustomInteractorStyle::OnRightButtonDown(){
    // 记录按下的位置
    this->GetInteractor()->GetEventPosition(this->RightButtonDownPosition);
    this->RightButtonPressed = true;
}

void CustomInteractorStyle::OnRightButtonUp(){
    int endPosition[2]; // 存储右键松开时的坐标
    this->GetInteractor()->GetEventPosition(endPosition);

    // 结束平移
    this->EndPan();

    int dx = endPosition[0] - this->RightButtonDownPosition[0];
    int dy = endPosition[1] - this->RightButtonDownPosition[1];
    double distance = sqrt(dx * dx + dy * dy);

    // 如果是shift加右键，则弹出菜单栏
    if (this->RightButtonPressed && distance < 5.0) {
        // this->PopupMenu();
    }

    // 不管是单击还是拖拽，结束时都需要重置RightButtonPressed
    this->RightButtonPressed = false;
}

void CustomInteractorStyle::OnMouseMove(){
    // 检查右键是否被按下，用于平移（右键逻辑）
    if (this->RightButtonPressed) {
        int currentPosition[2];
        this->GetInteractor()->GetEventPosition(currentPosition);
        int dx = currentPosition[0] - this->RightButtonDownPosition[0];
        int dy = currentPosition[1] - this->RightButtonDownPosition[1];
        double distance = sqrt(dx * dx + dy * dy);

        if (distance >= 5.0) { // 如果移动距离超过阈值，认为是拖拽操作
            // 执行平移操作
            vtkInteractorStyleTrackballCamera::OnMouseMove();
        }
        // 如果移动距离小，不进行操作，等待OnRightButtonUp中进一步处理
    }
}
