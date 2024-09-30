#include "cpcsnode.h"

CPcsNode::CPcsNode() {}

vtkSmartPointer<vtkAxesActor> CPcsNode::draw(int x) {
    // 创建一个vtkAxes对象（坐标轴的数据源）
    vtkSmartPointer<vtkAxes> axes = vtkSmartPointer<vtkAxes>::New();

    // 创建一个变换，用于将坐标轴平移到新的坐标系位置
    vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
    transform->Translate(getPcs()->m_poso.x, getPcs()->m_poso.y, getPcs()->m_poso.z);

    // 使用vtkTransformFilter来应用变换到vtkAxes
    vtkSmartPointer<vtkTransformFilter> transformFilter = vtkSmartPointer<vtkTransformFilter>::New();
    transformFilter->SetInputConnection(axes->GetOutputPort());
    transformFilter->SetTransform(transform);

    // 创建一个新的vtkAxesActor，使用变换后的vtkAxes作为数据源
    vtkSmartPointer<vtkAxesActor> axesActor = vtkSmartPointer<vtkAxesActor>::New();
    axesActor->SetUserTransform(transform);

    // 设置 X Y Z 轴标题颜色为黄色，用于和全局坐标系区分
    axesActor->GetXAxisCaptionActor2D()->GetCaptionTextProperty()->SetColor(1, 1, 0);
    axesActor->GetYAxisCaptionActor2D()->GetCaptionTextProperty()->SetColor(1, 1, 0);
    axesActor->GetZAxisCaptionActor2D()->GetCaptionTextProperty()->SetColor(1, 1, 0);

    // 设置字体大小
    axesActor->GetXAxisCaptionActor2D()->GetCaptionTextProperty()->SetFontSize(10);
    axesActor->GetYAxisCaptionActor2D()->GetCaptionTextProperty()->SetFontSize(10);
    axesActor->GetZAxisCaptionActor2D()->GetCaptionTextProperty()->SetFontSize(10);

    axesActor->SetTotalLength(0.4, 0.4, 0.4); // 设置轴的长度
    axesActor->SetConeRadius(0.1); // 设置轴锥体的半径
    axesActor->SetCylinderRadius(0.1); // 设置轴圆柱体的半径
    axesActor->SetSphereRadius(0.05); // 设置轴末端的球体半径

    return axesActor;
}

int CPcsNode::nPcsNodeCount = 0;
