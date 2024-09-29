#include "cpcsnode.h"

CPcsNode::CPcsNode() {}

vtkSmartPointer<vtkAxesActor> CPcsNode::draw(){
      // 创建坐标器
    auto axes = vtkSmartPointer<vtkAxesActor>::New();
    axes->SetPosition(getPcs()->m_poso.x, getPcs()->m_poso.y, getPcs()->m_poso.z);

    // 设置 X Y Z 轴标题颜色为黑色
    axes->GetXAxisCaptionActor2D()->GetCaptionTextProperty()->SetColor(0.0, 0.0, 0.0);
    axes->GetYAxisCaptionActor2D()->GetCaptionTextProperty()->SetColor(0.0, 0.0, 0.0);
    axes->GetZAxisCaptionActor2D()->GetCaptionTextProperty()->SetColor(0.0, 0.0, 0.0);

    // 设置字体大小
    axes->GetXAxisCaptionActor2D()->GetCaptionTextProperty()->SetFontSize(10);
    axes->GetYAxisCaptionActor2D()->GetCaptionTextProperty()->SetFontSize(10);
    axes->GetZAxisCaptionActor2D()->GetCaptionTextProperty()->SetFontSize(10);

    axes->SetTotalLength(0.4, 0.4, 0.4); // 设置轴的长度
    axes->SetConeRadius(0.1); // 设置轴锥体的半径
    axes->SetCylinderRadius(0.1); // 设置轴圆柱体的半径
    axes->SetSphereRadius(0.05); // 设置轴末端的球体半径

    return axes;
}

int CPcsNode::nPcsNodeCount = 0;
