#include "clickhighlightstyle.h"
#include <QDebug>

// 实现New()方法的宏
vtkStandardNewMacro(MouseInteractorHighlightActor);

MouseInteractorHighlightActor::MouseInteractorHighlightActor(vtkInteractorStyleTrackballCamera* parent)
{

}

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
    double* pos = picker->GetPickPosition(); // 用于存储拾取点的世界坐标

    // 创建vtkCellPicker拾取器
    // vtkSmartPointer<vtkCellPicker> picker = vtkSmartPointer<vtkCellPicker>::New();
    // picker->SetTolerance(0.0005); // 设置拾取容差
    // picker->Pick(clickPos[0], clickPos[1], 0, renderer);
    // picker->SetTolerance(0.005); // 设置拾取容差
    // double* pos = picker->GetPickPosition(); // 用于存储拾取点的世界坐标

    // vtkIdType cellId = picker->GetCellId(); // 拾取到的单元格ID
    // int subId = picker->GetSubId(); // 用于存储拾取到的单元格的子ID（例如，多边形的一个顶点）。
    // double* pcoords = picker->GetPCoords(); // 用于存储拾取点在单元格参数坐标系中的位置
    // double dist2; // 用于存储拾取点到最近的单元格的距离的平方

    // 如果选中了actor
    if (newPickedActor)
    {
        // 如果拾取成功，输出拾取点的世界坐标
        m_pMainWin->getChosenListMgr()->CreatPosition(pos); // 将选中的坐标传入管理器

        // 生成一个用于高亮的顶点，并存入pickedActors
        auto actor = CreatHighLightPoint(pos);

        vtkSmartPointer<vtkProperty> originalProperty = vtkSmartPointer<vtkProperty>::New();
        originalProperty->DeepCopy(actor->GetProperty());
        pickedActors.emplace_back(actor, originalProperty);// emplace_back作用等于push_back
        HighlightActor(actor);

        // 存储选中的actor的原始属性
        // vtkSmartPointer<vtkProperty> originalProperty = vtkSmartPointer<vtkProperty>::New();
        // originalProperty->DeepCopy(newPickedActor->GetProperty());

        // pickedActors.emplace_back(newPickedActor, originalProperty);// emplace_back作用等于push_back
        // HighlightActor(newPickedActor);
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
    double* pos = picker->GetPickPosition(); // 用于存储拾取点的世界坐标

    // 如果选中了actor
    if(newpickedActor){
        // 如果拾取成功，输出拾取点的世界坐标
        m_pMainWin->getChosenListMgr()->DeletePosition(pos);

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

//设置渲染器
void MouseInteractorHighlightActor::SetRenderer(vtkRenderer * renderer)
{
    this->renderer = renderer;
}

vtkActor* MouseInteractorHighlightActor::CreatHighLightPoint(double pos[3])
{
    // 生成一个用于高亮的顶点，表示选中的点
    auto points = vtkSmartPointer<vtkPoints>::New();
    points->InsertNextPoint(pos);

    // 创建几何图形容器并设置点集
    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);

    // 创建一个顶点,用过滤器将提取的点转化为更好观察的图形(glyph)，改善可视化效果
    vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
    glyphFilter->SetInputData(polyData);
    glyphFilter->Update();

    // 创建映射器
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(glyphFilter->GetOutputPort());

    // 创建执行器，添加mapper
    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetPointSize(7); // 设置点的大小
    point_actors.push_back(actor); // 存入point_actors

    renderer->AddActor(actor);
    return actor;
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

// 恢复到点击选中的状态
void MouseInteractorHighlightActor::BackChoosen(vtkActor *actor)
{
    HighlightActor(actor); // 恢复高亮显示
}

QVector<std::pair<vtkSmartPointer<vtkActor>, vtkSmartPointer<vtkProperty> > > &MouseInteractorHighlightActor::getPickedActors()
{
    return pickedActors;
}
