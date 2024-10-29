#include "clickhighlightstyle.h"
#include <QDebug>

// 实现New()方法的宏
vtkStandardNewMacro(MouseInteractorHighlightActor);

// 实现左键按下事件的处理方法
void MouseInteractorHighlightActor::OnLeftButtonDown()
{
    // 获取鼠标点击的位置
    int* clickPos = this->GetInteractor()->GetEventPosition();

    // 创建一个PropPicker对象，用于选择点击位置的actor
    vtkSmartPointer<vtkPropPicker> picker = vtkSmartPointer<vtkPropPicker>::New();
    picker->Pick(clickPos[0], clickPos[1], 0, renderer);
    // 获取选中的actor
    vtkActor* newPickedActor = picker->GetActor();

    // 如果选中了actor
    if (newPickedActor)
    {
        // 存储选中的actor的原始属性
        vtkSmartPointer<vtkProperty> originalProperty = vtkSmartPointer<vtkProperty>::New();
        originalProperty->DeepCopy(newPickedActor->GetProperty());

        pickedActors.emplace_back(newPickedActor, originalProperty);// emplace_back作用等于push_back
        HighlightActor(newPickedActor);
    }

    // 调用基类的左键按下事件
    vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
}


// 重写右键按下事件，取消选中
void MouseInteractorHighlightActor::OnRightButtonDown()
{
    // 获取鼠标点击的位置
    int* clickPos = this->GetInteractor()->GetEventPosition();
    // 创建一个PropPicker对象，用于选择点击位置的actor
    vtkSmartPointer<vtkPropPicker> picker = vtkSmartPointer<vtkPropPicker>::New();
    picker->Pick(clickPos[0], clickPos[1], clickPos[2], renderer);
    // 获取选中的actor并取消高亮
    vtkActor* newpickedActor = picker->GetActor();

    // 如果选中了actor
    if(newpickedActor){
        // 遍历存储的PickedActors，寻找并恢复被选中的actor的属性
        for (auto item = pickedActors.begin(); item != pickedActors.end();item++)
        {
            // 如果选中的是PickedActors中的actor
            if (item->first == newpickedActor)
            {
                ResetActor(newpickedActor); // 恢复actor的属性
                item = pickedActors.erase(item); // 从列表中移除该actor
                break; // 找到并恢复后退出循环
            }
        }
    }
    // 如果选中的是空白，则取消全部高亮
    else {
        // 遍历PickedActors，恢复被选中的actor的属性
        for (auto item = pickedActors.begin(); item != pickedActors.end();item++)
        {
            ResetActor(item->first); // 恢复actor的属性
        }
        pickedActors.clear();
    }

    // 调用基类的右键按下事件处理方法
    vtkInteractorStyleTrackballCamera::OnRightButtonDown();
}

void MouseInteractorHighlightActor::SetRenderer(vtkRenderer * renderer)
{
    this->renderer = renderer;
}

// 实现高亮显示actor的方法
void MouseInteractorHighlightActor::HighlightActor(vtkActor* actor)
{
    // 设置actor的颜色为红色，并调整其漫反射和镜面反射属性
    actor->GetProperty()->SetColor(1.0, 0.0, 0.0);
    // actor->GetProperty()->SetDiffuse(1.0);
    actor->GetProperty()->SetSpecular(0.0);

}

// 实现恢复actor属性的方法
void MouseInteractorHighlightActor::ResetActor(vtkActor* actor)
{
    for (const auto& pair : pickedActors)
    {
        // 如果选中的是PickedActors中的actor
        if (pair.first == actor)
        {
            actor->GetProperty()->DeepCopy(pair.second);
            break;
        }
    }
}

QVector<std::pair<vtkSmartPointer<vtkActor>, vtkSmartPointer<vtkProperty> > > &MouseInteractorHighlightActor::getPickedActors()
{
    return pickedActors;
}
