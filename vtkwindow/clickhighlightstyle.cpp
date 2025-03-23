#include "clickhighlightstyle.h"

// 实现New()方法的宏
vtkStandardNewMacro(MouseInteractorHighlightActor);

MouseInteractorHighlightActor::MouseInteractorHighlightActor(vtkInteractorStyleTrackballCamera* parent)
{
    vtkMenu = new QMenu(m_pMainWin); // 创建菜单
}

// 实现左键按下事件的处理方法
void MouseInteractorHighlightActor::OnLeftButtonDown()
{
    vtkMenu->hideTearOffMenu(); // 隐藏菜单栏
    // 获取鼠标点击的位置
    int* clickPos = this->GetInteractor()->GetEventPosition();

    // 创建一个PropPicker对象，用于选择点击位置的actor
    vtkSmartPointer<vtkPropPicker> picker = vtkSmartPointer<vtkPropPicker>::New();
    picker->Pick(clickPos[0], clickPos[1], 0, renderer);

    // 创建一个CellPicker对象，用于选择点击位置的actor
    vtkSmartPointer<vtkCellPicker> cellPicker = vtkSmartPointer<vtkCellPicker>::New();
    cellPicker->SetTolerance(1e-06); // 设置拾取精度
    cellPicker->Pick(clickPos[0], clickPos[1], 0, renderer);

    // 获取选中的actor
    vtkActor* newPickedActor = picker->GetActor();
    double* pos = picker->GetPickPosition(); // 用于存储拾取点的世界坐标

    // 如果选中了actor
    if (newPickedActor)
    {
        // 得到点中的centity
        QMap<vtkSmartPointer<vtkActor>, CEntity*>& actorToEntity=m_pMainWin->getactorToEntityMap();
        CEntity* entity=actorToEntity[newPickedActor];

        // 只有打开识别点才会记录坐标
        if(m_pMainWin->getPWinToolWidget()->IsFindPoint_Checked()){
            m_pMainWin->getChosenListMgr()->CreatPosition(pos); // 将选中的坐标传入管理器
            auto actor = CreatHighLightPoint(pos);// 生成一个用于高亮的顶点，并存入pickedActors
            HighlightActor(actor);
        }

        // 如果选中的是点云类型的actor
        if(entity->GetUniqueType() == enPointCloud){
            auto cloudEntity = (CPointCloud*)entity;
            QString log = QString("已选中点云: ") + cloudEntity->GetObjectAutoName();
            m_pMainWin->getPWinVtkPresetWidget()->setWidget(log);
            // 给拟合用的cloudptr赋值
            m_pMainWin->getpWinFileMgr()->cloudptr = cloudEntity->m_pointCloud.makeShared();

            // 用于判断当前是否开启识别功能
            auto findState = m_pMainWin->getPWinToolWidget()->getAction_Checked();
            if(findState){
                if(findState->objectName() == "识别点"){
                    m_pMainWin->getPWinToolWidget()->onFindPoint();
                }
                if(findState->objectName() == "识别线"){
                    m_pMainWin->getPWinToolWidget()->onFindLine();
                }
                if(findState->objectName() == "识别圆"){
                    m_pMainWin->getPWinToolWidget()->onFindCircle();
                }
                if(findState->objectName() == "识别平面"){
                    m_pMainWin->getPWinToolWidget()->onFindPlane();
                }
                if(findState->objectName() == "识别球形"){
                    m_pMainWin->getPWinToolWidget()->onFindSphere();
                }
                if(findState->objectName() == "识别圆锥"){
                    m_pMainWin->getPWinToolWidget()->onFindCone();
                }
                if(findState->objectName() == "识别圆柱"){
                    m_pMainWin->getPWinToolWidget()->onFindCylinder();
                }
                if(findState->objectName() == "识别矩形"){
                    m_pMainWin->getPWinToolWidget()->onFindPlane();
                }
            }
        }
    }

    // 调用基类的左键按下事件
    vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
}

void MouseInteractorHighlightActor::OnLeftButtonUp()
{
    // 调用基类的左键释放事件
    vtkInteractorStyleTrackballCamera::OnLeftButtonUp();
}

// 重写右键按下事件，取消选中；双击则弹出菜单栏
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
        // 清除选中的点
        m_pMainWin->getChosenListMgr()->getChosenActorAxes().clear(); // 清除所有选中的点的记录

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
        DeleteHighLightPoint();
    }
    // 如果选中的是空白，则取消全部高亮，并清除选中点在chosenlist的记录
    else {
        CancelHighlightActors();
        m_pMainWin->getChosenListMgr()->getChosenActorAxes().clear(); // 清除所有选中的点的记录
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

void MouseInteractorHighlightActor::DeleteHighLightPoint()
{
    // 获取渲染器中的所有 actor
    auto* actorCollection = renderer->GetActors();

    // 创建一个迭代器用于遍历actor集合
    vtkCollectionSimpleIterator it;

    // 初始化迭代器，准备遍历actor集合
    actorCollection->InitTraversal(it);

    vtkActor* actor;
    // 如果渲染器中有用于高亮的顶点，即point_actors不为空，则从窗口中移除
    while ((actor = actorCollection->GetNextActor(it)) != nullptr){
        for(auto i = 0;i != point_actors.size();i++){
            if(point_actors[i] == actor)
                renderer->RemoveActor(actor);
        }
        renderer->Modified(); // 更新渲染器
    }
}

void MouseInteractorHighlightActor::CancelHighlightActors()
{
    // 遍历PickedActors，恢复被选中的actor的属性
    for (auto item = pickedActors.begin(); item != pickedActors.end();item++)
    {
        ResetActor(item->first); // 恢复actor的属性
    }
    DeleteHighLightPoint();
    renderer->Render();
}

// 实现高亮显示actor的方法
void MouseInteractorHighlightActor::HighlightActor(vtkActor* actor)
{
    // 如果actor没被隐藏
    if(actor != nullptr){
        // 保存高亮前的属性
        vtkSmartPointer<vtkProperty> originalProperty = vtkSmartPointer<vtkProperty>::New();
        originalProperty->DeepCopy(actor->GetProperty());
        pickedActors.emplace_back(actor, originalProperty);// emplace_back作用等于push_back
        // 让actor高亮，并放在窗口最上层
        actor->GetProperty()->SetColor(MainWindow::HighLightColor[0], MainWindow::HighLightColor[1],
                                       MainWindow::HighLightColor[2]);
        if (renderer != nullptr){
            // 将actor从渲染器中移除并重新添加
            renderer->RemoveActor(actor);
            renderer->AddActor(actor);
            renderer->Render();
        }
    }

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

QVector<std::pair<vtkSmartPointer<vtkActor>, vtkSmartPointer<vtkProperty>>> &MouseInteractorHighlightActor::getPickedActors()
{
    return pickedActors;
}

QVector<vtkActor *> MouseInteractorHighlightActor::getpoint_actors()
{
    return  point_actors;
}

void MouseInteractorHighlightActor::onCancelSelect()
{
    OnRightButtonDown();
}
