#include "vtkwindow/vtkwidget.h"
#include <QFileDialog>  // 用于文件对话框
#include <QOpenGLContext>
#include <qopenglfunctions.h>
#include <QMessageBox>

#include <vtkInteractorStyle.h>
#include <vtkEventQtSlotConnect.h>


VtkWidget::VtkWidget(QWidget *parent)
    : QWidget(parent),
    cloud1(new pcl::PointCloud<pcl::PointXYZ>()),  // 初始化第一个点云对象
    cloud2(new pcl::PointCloud<pcl::PointXYZ>()), // 初始化第二个点云对象
    comparisonCloud(new pcl::PointCloud<pcl::PointXYZRGB>()), // 初始化比较好的点云对象
    alignedCloud(new pcl::PointCloud<pcl::PointXYZRGB>())
{
    vtkObject::GlobalWarningDisplayOff();// 禁用 VTK 的错误处理弹窗
    m_pMainWin = (MainWindow*) parent;

    QVBoxLayout *mainlayout = new QVBoxLayout(this);
    mainlayout->setContentsMargins(0, 0, 0, 0); // 去除布局的边距
    mainlayout->setSpacing(0); // 去除布局内部的间距
    setUpVtk(mainlayout); // 配置vtk窗口
    this->setLayout(mainlayout);

}

void VtkWidget::setUpVtk(QVBoxLayout *layout){
    // 初始化渲染器和交互器
    renderer = vtkSmartPointer<vtkRenderer>::New();
    renderer->SetBackground(1, 1, 1); // 设置渲染器颜色为白
    renWin = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    renWin->AddRenderer(renderer);  // 将渲染器添加到渲染窗口

    // 创建QVTKOpenGLNativeWidget作为渲染窗口
    vtkWidget = new QVTKOpenGLNativeWidget();
    layout->addWidget(vtkWidget);
    vtkWidget->setRenderWindow(renWin);

    // 添加高亮样式
    m_highlightstyle = vtkSmartPointer<MouseInteractorHighlightActor>::New();
    m_highlightstyle->SetRenderer(renderer);
    m_highlightstyle->SetUpMainWin(m_pMainWin);
    renWin->GetInteractor()->SetInteractorStyle(m_highlightstyle);

    createAxes();// 创建左下角全局坐标系

    // 创建初始视角相机
    vtkCamera* camera = renderer->GetActiveCamera();
    if (camera) {
        // 设置相机的初始位置和焦点
        camera->SetPosition(0, 0, 1);
        camera->SetFocalPoint(0, 0, 0);
        camera->SetViewUp(0, 1, 0);
        // camera->SetViewAngle(60); // 调整视野角度
        // camera->SetClippingRange(0.1, 1000);// 根据场景的具体大小和需要调整裁剪范围
        camera->OrthogonalizeViewUp(); // 确保与SetViewUp方向正交

        renderer->ResetCamera();
        renWin->Render();
    }

    getRenderWindow()->Render();
}

void VtkWidget::OnMouseMove()
{
    if (!isDragging) {
        return;
    }

    int clickPos[2];
    renWin->GetInteractor()->GetEventPosition(clickPos);

    // 更新当前拖动的文本演员
    infoTextActor->SetPosition(clickPos[0] - 30, clickPos[1] - 20);
    double* a = infoTextActor->GetPosition();

    // 更新对应的文本框位置
    rectangleActor->SetPosition(a[0], a[1]);

    // 更新对应的指向线段
    Linechange();

    // 重新渲染窗口
    getRenderWindow()->Render();
}

void VtkWidget::OnLeftButtonPress()
{
    int* clickPos = renWin->GetInteractor()->GetEventPosition();

    // 遍历所有文本框，检查点击位置是否在某个文本框内
    for (auto it = entityToTextActors.begin(); it != entityToTextActors.end(); ++it)
    {
        vtkSmartPointer<vtkTextActor> textActor = it.value();

        double bbox[4];
        textActor->GetBoundingBox(renderer, bbox);
        double textWidth = bbox[1] - bbox[0];
        double textHeight = bbox[3] - bbox[2];

        double width = textWidth + 20; // 加上边距
        double height = textHeight + 10; // 加上边距
        double* position = textActor->GetPosition();

        if (clickPos[0] >= position[0] && clickPos[0] <= position[0] + width &&
            clickPos[1] >= position[1] && clickPos[1] <= position[1] + height)
        {
            // 设置当前拖动的文本框
            infoTextActor = textActor;
            rectangleActor = entityToTextBoxs[it.key()];
            iconActor = entityToIcons[it.key()];
            lineActor = entityToLines[it.key()];

            isDragging = true; // 开启拖动状态
            renWin->GetInteractor()->SetEventInformation(clickPos[0], clickPos[1], 0, 0, 0, 0);
            renWin->GetInteractor()->SetInteractorStyle(0);
            return;
        }
    }
}

void VtkWidget::OnLeftButtonRelease()
{
    isDragging = false; // 关闭拖动状态
    renWin->GetInteractor()->SetInteractorStyle(m_highlightstyle);
}

void VtkWidget::setCentity(CEntity* entity)
{
    if (!entity) return;

    // 如果已经存在该实体的信息，直接返回
    if (entityToTextActors.contains(entity)) return;

    elementEntity = entity;
    createText(entity);
}

vtkSmartPointer<vtkTextActor> &VtkWidget::getInfoText()
{
    return infoTextActor;
}

void VtkWidget::createText(CEntity* entity)
{
    vtkSmartPointer<vtkTextActor> textActor = vtkSmartPointer<vtkTextActor>::New();
    textActor->GetTextProperty()->SetFontSize(16);
    textActor->GetTextProperty()->SetFontFamilyToTimes();
    textActor->GetTextProperty()->SetColor(MainWindow::InfoTextColor);
    textActor->GetTextProperty()->SetJustificationToLeft();
    textActor->GetTextProperty()->SetBold(1);
    textActor->GetTextProperty()->SetShadow(true);

    QString qstr = entity->getCEntityInfo();
    QByteArray byteArray = qstr.toUtf8();
    textActor->SetInput(byteArray.constData());

    // 计算文本框的初始位置
    double x = renWin->GetSize()[0] - 200 - increaseDis[0];
    double y = renWin->GetSize()[1] - 100 - increaseDis[1];
    textActor->SetPosition(x, y);
    increaseDis[1] += 100;

    // 检查是否重叠，并调整位置
    double bbox[4];
    textActor->GetBoundingBox(renderer, bbox);
    double textWidth = bbox[1] - bbox[0];
    double textHeight = bbox[3] - bbox[2];
    double width = textWidth + 20;
    double height = textHeight + 10;

    // 检查是否与其他文本框重叠
    for (auto it = entityToTextActors.begin(); it != entityToTextActors.end(); ++it)
    {
        double otherBbox[4];
        it.value()->GetBoundingBox(renderer, otherBbox);
        if (x + width > otherBbox[0] && x < otherBbox[1] &&
            y + height > otherBbox[2] && y < otherBbox[3])
        {
            // 如果重叠，调整位置
            y -= (height + 10); // 向上移动
        }
    }

    // 创建文本框
    vtkSmartPointer<vtkActor2D> textBox = createTextBox(textActor, x, y);

    // 创建指向线段
    vtkSmartPointer<vtkActor2D> line = createLine(entity, textActor);

    // 存储信息
    entityToTextActors[entity] = textActor;
    entityToTextBoxs[entity] = textBox;
    entityToLines[entity] = line;

    // 添加到渲染器
    renderer->AddActor(textActor);
    renderer->AddActor(textBox);
    renderer->AddActor(line);

    // 设置事件回调
    renWin->GetInteractor()->AddObserver(vtkCommand::LeftButtonPressEvent, this, &VtkWidget::OnLeftButtonPress);
    renWin->GetInteractor()->AddObserver(vtkCommand::MouseMoveEvent, this, &VtkWidget::OnMouseMove);
    renWin->GetInteractor()->AddObserver(vtkCommand::LeftButtonReleaseEvent, this, &VtkWidget::OnLeftButtonRelease);

    renWin->Render();
}

// 创建文本框
vtkSmartPointer<vtkActor2D> VtkWidget::createTextBox(vtkSmartPointer<vtkTextActor> textActor, double x, double y)
{
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    double bbox[4];
    textActor->GetBoundingBox(renderer, bbox);
    double textWidth = bbox[1] - bbox[0];
    double textHeight = bbox[3] - bbox[2];
    double width = textWidth + 20;
    double height = textHeight + 10;

    points->InsertNextPoint(0, 0, 0);
    points->InsertNextPoint(width, 0, 0);
    points->InsertNextPoint(width, height, 0);
    points->InsertNextPoint(0, height, 0);

    vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
    vtkIdType lineIds[2];
    lineIds[0] = 0; lineIds[1] = 1;
    lines->InsertNextCell(2, lineIds);
    lineIds[0] = 1; lineIds[1] = 2;
    lines->InsertNextCell(2, lineIds);
    lineIds[0] = 2; lineIds[1] = 3;
    lines->InsertNextCell(2, lineIds);
    lineIds[0] = 3; lineIds[1] = 0;
    lines->InsertNextCell(2, lineIds);

    vtkSmartPointer<vtkCellArray> polygons = vtkSmartPointer<vtkCellArray>::New();
    vtkIdType ids[4] = {0, 1, 2, 3};
    polygons->InsertNextCell(4, ids);

    vtkSmartPointer<vtkPolyData> rectangle = vtkSmartPointer<vtkPolyData>::New();
    rectangle->SetPoints(points);
    rectangle->SetLines(lines);
    rectangle->SetPolys(polygons);

    vtkSmartPointer<vtkPolyDataMapper2D> rectangleMapper = vtkSmartPointer<vtkPolyDataMapper2D>::New();
    rectangleMapper->SetInputData(rectangle);

    vtkSmartPointer<vtkActor2D> textBox = vtkSmartPointer<vtkActor2D>::New();
    textBox->SetMapper(rectangleMapper);
    textBox->GetProperty()->SetColor(0.1, 0.2, 0.3);
    textBox->GetProperty()->SetOpacity(0.2);
    textBox->GetProperty()->SetLineWidth(2);
    textBox->SetPosition(x, y);

    return textBox;
}

vtkSmartPointer<vtkActor2D> VtkWidget::createLine(CEntity* entity, vtkSmartPointer<vtkTextActor> textActor)
{
    vtkSmartPointer<vtkPNGReader> pngReader = vtkSmartPointer<vtkPNGReader>::New();
    double* a = textActor->GetPosition();
    CPosition b;
    QString filename;

    if (entity->getEntityType() == enDistance)
    {
        CDistance* dis = static_cast<CDistance*>(entity);
        double distance = dis->getdistanceplane();
        CObject* obj11 = dis->parent[1];
        CPlane* plane = static_cast<CPlane*>(obj11);
        QVector4D plane_normal = plane->getNormal();
        plane_normal.normalize();
        b.x = dis->getbegin().x - (distance * plane_normal.x()) / 2;
        b.y = dis->getbegin().y - (distance * plane_normal.y()) / 2;
        b.z = dis->getbegin().z - (distance * plane_normal.z()) / 2;
        filename = ":/component/construct/distance.png";
    }
    else if (entity->getEntityType() == enPoint)
    {
        CPoint* point = static_cast<CPoint*>(entity);
        b = point->GetPt();
        filename = ":/component/find/point.jpg";
    }
    else if (entity->getEntityType() == enLine)
    {
        CLine* line = static_cast<CLine*>(entity);
        b = line->GetObjectCenterLocalPoint();
        filename = ":/component/find/line.jpg";
    }
    else if (entity->getEntityType() == enCircle)
    {
        CCircle* circle = static_cast<CCircle*>(entity);
        b = circle->getCenter();
        filename = ":/component/find/circle.jpg";
    }
    else if (entity->getEntityType() == enSphere)
    {
        CSphere* s = static_cast<CSphere*>(entity);
        b = s->getCenter();
        filename = ":/component/find/sphere.jpg";
    }
    else if (entity->getEntityType() == enPlane)
    {
        CPlane* s = static_cast<CPlane*>(entity);
        b = s->getCenter();
        filename = ":/component/find/plan.jpg";
    }
    else if (entity->getEntityType() == enCylinder)
    {
        CCylinder* s = static_cast<CCylinder*>(entity);
        b = s->getBtm_center();
        filename = ":/component/find/cylinder.jpg";
    }
    else if (entity->getEntityType() == enCone)
    {
        CCone* s = static_cast<CCone*>(entity);
        b = s->getVertex();
        filename = ":/component/find/cone.jpg";
    }

    entityToEndPoints[entity] = b;

    vtkSmartPointer<vtkCoordinate> coordinate = vtkSmartPointer<vtkCoordinate>::New();
    coordinate->SetValue(b.x, b.y, b.z);
    coordinate->SetCoordinateSystemToWorld();
    int* viewportMidPoint = coordinate->GetComputedViewportValue(renderer);

    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    points->InsertNextPoint(a[0], a[1], 0.0);
    points->InsertNextPoint(viewportMidPoint[0], viewportMidPoint[1], viewportMidPoint[2]);

    vtkSmartPointer<vtkPolyData> linePolyData = vtkSmartPointer<vtkPolyData>::New();
    linePolyData->SetPoints(points);

    vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
    lines->InsertNextCell(2);
    lines->InsertCellPoint(0);
    lines->InsertCellPoint(1);
    linePolyData->SetLines(lines);

    vtkSmartPointer<vtkPolyDataMapper2D> lineMapper = vtkSmartPointer<vtkPolyDataMapper2D>::New();
    lineMapper->SetInputData(linePolyData);

    vtkSmartPointer<vtkActor2D> lineActor = vtkSmartPointer<vtkActor2D>::New();
    lineActor->SetMapper(lineMapper);
    lineActor->GetProperty()->SetColor(MainWindow::InfoTextColor);
    lineActor->GetProperty()->SetLineWidth(2);

    return lineActor;
}

void VtkWidget::Linechange()
{
    if (!isDragging || !infoTextActor || !lineActor) {
        return; // 如果没有拖动或相关对象为空，则直接返回
    }

    // 获取当前拖动的文本框位置
    double* textPosition = infoTextActor->GetPosition();

    // 获取指向线段的终点位置（从实体到终点映射中获取）
    CPosition endPoint = entityToEndPoints[getEntityFromTextActor(infoTextActor)];

    // 将终点从世界坐标转换为视口坐标
    vtkSmartPointer<vtkCoordinate> coordinate = vtkSmartPointer<vtkCoordinate>::New();
    coordinate->SetValue(endPoint.x, endPoint.y, endPoint.z);
    coordinate->SetCoordinateSystemToWorld();
    int* viewportMidPoint = coordinate->GetComputedViewportValue(renderer);

    // 更新指向线段的起点和终点
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    points->InsertNextPoint(textPosition[0], textPosition[1], 0.0); // 起点为文本框位置
    points->InsertNextPoint(viewportMidPoint[0], viewportMidPoint[1], viewportMidPoint[2]); // 终点为视口坐标

    linePolyData->SetPoints(points);

    // 更新线段的映射数据
    lineMapper->SetInputData(linePolyData);
    lineMapper->Update();
    lineActor->SetMapper(lineMapper);

    // 重新渲染窗口
    renWin->Render();
}

void VtkWidget::closeText()
{
    for (auto it = entityToTextActors.begin(); it != entityToTextActors.end(); ++it)
    {
        renderer->RemoveActor(it.value());
    }
    for (auto it = entityToTextBoxs.begin(); it != entityToTextBoxs.end(); ++it)
    {
        renderer->RemoveActor(it.value());
    }
    for (auto it = entityToLines.begin(); it != entityToLines.end(); ++it)
    {
        renderer->RemoveActor(it.value());
    }
    for (auto it = entityToIcons.begin(); it != entityToIcons.end(); ++it)
    {
        renderer->RemoveActor(it.value());
    }

    entityToTextActors.clear();
    entityToTextBoxs.clear();
    entityToLines.clear();
    entityToIcons.clear();

    renWin->Render();
}

CEntity* VtkWidget::getEntityFromTextActor(vtkSmartPointer<vtkTextActor> textActor)
{
    for (auto it = entityToTextActors.begin(); it != entityToTextActors.end(); ++it)
    {
        if (it.value() == textActor)
        {
            return it.key();
        }
    }
    return nullptr;
}

void VtkWidget::ShowColorBar(double minDistance, double maxDistance){
    // 创建一个 PolyData 对象来存储色温尺的几何信息
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkCellArray> lines = vtkSmartPointer<vtkCellArray>::New();
    vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
    colors->SetNumberOfComponents(3); // RGB

    // 定义色温尺的起点和终点
    int barHeight = 20; // 色温尺的高度
    int barWidth = 200; // 色温尺的宽度
    auto Width = renWin->GetSize()[0];
    auto Height = renWin->GetSize()[1];
    points->InsertNextPoint(Width-barWidth-10, 10, 0); // 起点
    points->InsertNextPoint(Width-barWidth, 10, 0); // 终点

    // 插入线段
    vtkIdType pointIds[2] = {0, 1};
    lines->InsertNextCell(2, pointIds);

    // 为每个顶点设置颜色
    colors->InsertTuple3(0, 0, 0, 255); // 左下角
    colors->InsertTuple3(1, 255, 0, 0); // 右下角
    colors->InsertTuple3(2, 255, 0, 0); // 右上角
    colors->InsertTuple3(3, 0, 0, 255); // 左上角

    // 创建 PolyData
    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);
    polyData->SetLines(lines);

    // 将颜色数据绑定到线段上（通过绘制多边形来模拟颜色条）
    vtkSmartPointer<vtkPolyData> colorBarPolyData = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkPoints> colorBarPoints = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkCellArray> colorBarPolys = vtkSmartPointer<vtkCellArray>::New();

    colorBarPoints->InsertNextPoint(Width - barWidth - 20, 10, 0);
    colorBarPoints->InsertNextPoint(Width - 20, 10, 0);
    colorBarPoints->InsertNextPoint(Width - 20, 10 + barHeight, 0);
    colorBarPoints->InsertNextPoint(Width - barWidth - 20, 10 + barHeight, 0);

    vtkIdType polyIds[4] = {0, 1, 2, 3};
    colorBarPolys->InsertNextCell(4, polyIds);

    colorBarPolyData->SetPoints(colorBarPoints);
    colorBarPolyData->SetPolys(colorBarPolys);
    colorBarPolyData->GetPointData()->SetScalars(colors); // 设置颜色数据

    // 创建 Mapper
    vtkSmartPointer<vtkPolyDataMapper2D> colorBarMapper = vtkSmartPointer<vtkPolyDataMapper2D>::New();
    colorBarMapper->SetInputData(colorBarPolyData);

    // 创建 Actor2D
    colorBarActor = vtkSmartPointer<vtkActor2D>::New();
    colorBarActor->SetMapper(colorBarMapper);

    // 添加最小值和最大值的文本标注
    vtkSmartPointer<vtkTextMapper> minTextMapper = vtkSmartPointer<vtkTextMapper>::New();
    minTextMapper->SetInput(std::to_string(minDistance).c_str());
    vtkSmartPointer<vtkActor2D> minTextActor = vtkSmartPointer<vtkActor2D>::New();
    minTextActor->SetMapper(minTextMapper);
    minTextActor->SetPosition(Width - barWidth - 100, barHeight + 7); // 调整位置以适应显示

    vtkSmartPointer<vtkTextMapper> maxTextMapper = vtkSmartPointer<vtkTextMapper>::New();
    maxTextMapper->SetInput(std::to_string(maxDistance).c_str());
    vtkSmartPointer<vtkActor2D> maxTextActor = vtkSmartPointer<vtkActor2D>::New();
    maxTextActor->SetMapper(maxTextMapper);
    maxTextActor->SetPosition(Width - 100, barHeight + 7); // 调整位置以适应显示
    minTextMapper->GetTextProperty()->SetFontSize(12);
    maxTextMapper->GetTextProperty()->SetFontSize(12);

    // 获取渲染器并添加色温条和标注
    renderer->AddActor2D(colorBarActor);
    renderer->AddActor2D(minTextActor);
    renderer->AddActor2D(maxTextActor);
    renderer->AddActor2D(colorBarActor);

    // 刷新渲染窗口
    renderer->Render();
}

vtkSmartPointer<vtkRenderWindow> VtkWidget::getRenderWindow(){
    return renWin;
}

vtkSmartPointer<vtkRenderer>& VtkWidget::getRenderer(){
    return renderer;
}

// 刷新vtk窗口
void VtkWidget::UpdateInfo(){
    reDrawCentity();
}

void VtkWidget::reDrawCentity(){
    auto entitylist = m_pMainWin->m_EntityListMgr->getEntityList();
    auto objectlist = m_pMainWin->m_ObjectListMgr->getObjectList();
    QMap<QString, bool> contentItemmap = m_pMainWin->getpWinFileMgr()->getContentItemMap();
    QMap<QString, bool> identifyItemmap = m_pMainWin->getpWinFileMgr()->getIdentifyItemMap();
    QVector<CEntity*> constructEntityList = m_pMainWin->getPWinToolWidget()->getConstructEntityList();//存储构建元素的列表
    QVector<CEntity*> identifyEntityList = m_pMainWin->getPWinToolWidget()->getIdentifyEntityList();//存储识别元素的列表
    QMap<vtkSmartPointer<vtkActor>, CEntity*>& actorToEntity = m_pMainWin->getactorToEntityMap();

    // 获取渲染器中的所有 actor
    auto* actorCollection = getRenderer()->GetViewProps();

    // 创建一个迭代器用于遍历actor集合
    vtkCollectionSimpleIterator it;

    // 初始化迭代器，准备遍历actor集合
    actorCollection->InitTraversal(it);

    vtkSmartPointer<vtkProp> prop;
    // 遍历并移除 vtkProp 对象，若有点云actor则跳过
    while ((prop = actorCollection->GetNextProp(it)) != nullptr)
    {
        renderer->RemoveViewProp(prop);
    }

    actorToEntity.clear();
    CPointCloud::getActorToPointCloud().clear();
    // 遍历entitylist绘制图形并加入渲染器
    for(auto i = 0;i < entitylist.size();i++){
        int constructFlag=0;
        int identifyFlag=0;

        // 循环判断是哪种元素
        for(int j=0;j<constructEntityList.size();j++){
            QString key=constructEntityList[j]->GetObjectCName() + "  " + constructEntityList[j]->GetObjectAutoName();
            if(entitylist[i] == constructEntityList[j]){//是构建的元素
                constructFlag=1;
                if(contentItemmap[key]){ // 如果不隐藏
                    vtkSmartPointer<vtkActor>actor = entitylist[i]->draw();
                    actorToEntity.insert(actor,entitylist[i]);
                    getRenderer()->AddActor(actor);
                    break;
                }
                else{ // 如果隐藏，则删除高亮前的记录
                    // m_highlightstyle->getPickedActors().erase()
                }
            }
        }

        //检查是否为识别的元素
        if(constructFlag==0){//不是构建的元素
            for(int j=0;j<identifyEntityList.size();j++){
                QString key=identifyEntityList[j]->GetObjectCName() + "  " + identifyEntityList[j]->GetObjectAutoName();
                if(entitylist[i] == identifyEntityList[j]){//是构建的元素
                    identifyFlag=1;
                    if(identifyItemmap[key]){ // 如果不隐藏
                        vtkSmartPointer<vtkActor>actor = entitylist[i]->draw();
                        actorToEntity.insert(actor,entitylist[i]);
                        getRenderer()->AddActor(actor);
                        break;
                    }
                }
            }
        }

        //既不是构建的元素也不是识别的元素
        if(constructFlag==0&&identifyFlag==0){
            vtkSmartPointer<vtkActor>actor=entitylist[i]->draw();
            actorToEntity.insert(actor,entitylist[i]);
            getRenderer()->AddActor(actor);
        }

        // 遍历objectlist绘制坐标系并加入渲染器
        for(auto object:objectlist){
            if(object)
                getRenderer()->AddActor(object->draw());
        }
    }
    renWin->Render(); // 刷新窗口
}

void VtkWidget::onHighLightActor(CEntity* entity)
{
    // 寻找对应的actor
    QMap<vtkSmartPointer<vtkActor>, CEntity*>& map = m_pMainWin->getactorToEntityMap();
    m_highlightstyle->HighlightActor(map.key(entity));
    renWin->Render();
}

// 创建全局坐标器
void VtkWidget::createAxes()
{
    // 创建并初始化全局坐标系
    auto axesActor = vtkSmartPointer<vtkAxesActor>::New();

    // 设置 X Y Z 轴标题颜色为黑色
    axesActor->GetXAxisCaptionActor2D()->GetCaptionTextProperty()->SetColor(0.0, 0.0, 0.0);
    axesActor->GetYAxisCaptionActor2D()->GetCaptionTextProperty()->SetColor(0.0, 0.0, 0.0);
    axesActor->GetZAxisCaptionActor2D()->GetCaptionTextProperty()->SetColor(0.0, 0.0, 0.0);

    axesActor->SetTotalLength(2, 2, 2); // 设置轴的长度
    axesActor->SetConeRadius(0.1); // 设置轴锥体的半径
    axesActor->SetCylinderRadius(0.1); // 设置轴圆柱体的半径
    axesActor->SetSphereRadius(0.05); // 设置轴末端的球体半径

    axeWidget = vtkSmartPointer<vtkOrientationMarkerWidget>::New();
    // 将坐标轴演员添加到orientationWidget
    axeWidget->SetOrientationMarker(axesActor);
    // 将orientationWidget与交互器关联
    axeWidget->SetInteractor(renWin->GetInteractor());
    // 设置视口
    axeWidget->SetViewport(0.0, 0.0, 0.2, 0.2);

    axeWidget->SetEnabled(1);
    axeWidget->InteractiveOn();
}

// 切换相机视角1
void VtkWidget::onTopView() {
    vtkCamera *camera = renderer->GetActiveCamera();
    if (camera) {
        camera->SetPosition(0, 0, 1);  // 重置相机位置为俯视
        camera->SetFocalPoint(0, 0, 0);
        camera->SetViewUp(0, 1, 0);
        camera->OrthogonalizeViewUp(); // 确保与SetVieswUp方向正交

        // 重新设置相机并渲染
        renderer->ResetCamera();
        //createText();
        renWin->Render();
    }
}

// 切换相机视角2
void VtkWidget::onRightView(){
    vtkCamera *camera = renderer->GetActiveCamera();
    if (camera) {
        camera->SetPosition(1, 0, 0);  // 重置相机位置为右侧
        camera->SetFocalPoint(0, 0, 0);
        camera->SetViewUp(0, 1, 0);
        camera->OrthogonalizeViewUp(); // 确保与SetViewUp方向正交

        // 重新设置相机并渲染
        renderer->ResetCamera();
        //createText();
        renWin->Render();
    }
}

// 切换相机视角3
void VtkWidget::onFrontView(){
    vtkCamera *camera = renderer->GetActiveCamera();
    if (camera) {
        camera->SetPosition(0, -1, 0);  // 重置相机位置为正视
        camera->SetFocalPoint(0, 0, 0);
        camera->SetViewUp(0, 0, 1);
        camera->OrthogonalizeViewUp(); // 确保与SetViewUp方向正交

        // 重新设置相机并渲染
        renderer->ResetCamera();
        //createText();
        renWin->Render();
    }
}

// 切换相机视角4，立体视角可以在前三个的基础上旋转
void VtkWidget::onIsometricView(){
    vtkCamera *camera = renderer->GetActiveCamera();
    if (camera) {
        camera->SetPosition(0, 0, 0); // 重置相机位置
        camera->SetViewUp(0, 1, 0);    // 重置视角向上方向
        camera->Azimuth(60);
        camera->Elevation(60);
        camera->OrthogonalizeViewUp();

        // 重新设置相机并渲染
        renderer->ResetCamera();
        //createText();
        renWin->Render();
    }
}

// 比较两个点云的处理函数
void VtkWidget::onCompare()
{
    auto entityList = m_pMainWin->m_EntityListMgr->getEntityList();
    auto cloudptr= m_pMainWin->getpWinFileMgr()->cloudptr;
    auto marksMap = m_pMainWin->getpWinFileMgr()->getContentItemMap();

    QVector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;
    for(int i=0;i<entityList.size();i++){
        CEntity* entity=entityList[i];
        if(!entity->IsSelected())continue;
        if(entity->GetUniqueType()==enPointCloud){
            auto & temp=((CPointCloud*)entity)->m_pointCloud;
            clouds.append(temp.makeShared());
        }
    }

    if(clouds.size()!=2){
        QString message="点云数目异常(只允许两个点云数据))";
        QMessageBox msgBox;
        msgBox.setWindowTitle("错误");
        msgBox.setText(message);
        msgBox.setIcon(QMessageBox::Critical); // 设置对话框图标为错误
        msgBox.setStandardButtons(QMessageBox::Ok); // 只显示“确定”按钮
        msgBox.exec(); // 显示对话框
        return ;
    }

    pcl::copyPointCloud( *clouds[0], *cloud1);
    pcl::copyPointCloud( *clouds[1], *cloud2);

    // 检查点云是否为空
    if (cloud1->empty() || cloud2->empty()) {
        QMessageBox::warning(this, "Warning", "其中一个或两个点云为空!");
        return;
    }

    // 创建KD-Tree用于点云2
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud2);

    // 将比较结果存在comparisonCloud，并设置点云大小
    comparisonCloud->clear(); // 清除上次比较的结果
    comparisonCloud->width = 0;
    comparisonCloud->height = 0;
    comparisonCloud->is_dense = false;
    comparisonCloud->resize(cloud1->size());

    // 初始化最大和最小距离变量
    float maxDistance = std::numeric_limits<float>::min();
    float minDistance = std::numeric_limits<float>::max();

    // 用于存储最近邻搜索的结果
    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1);

    // 遍历点云1中的每个点，找到与点云2中最近点的距离
    for (size_t i = 0; i < cloud1->size(); ++i) {
        if (kdtree.nearestKSearch(cloud1->at(i), 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
            float dist = std::sqrt(pointNKNSquaredDistance[0]);
            // 更新最大和最小距离
            if (dist > maxDistance) {
                maxDistance = dist;
            }
            if (dist < minDistance) {
                minDistance = dist;
            }
            // 设置比较结果点云的颜色和位置
            pcl::PointXYZRGB &point = comparisonCloud->at(i);
            float normalizedDistance = (dist - minDistance) / (maxDistance - minDistance); // 归一化距离到0-1之间
            int r = static_cast<int>(255 * normalizedDistance);
            int b = 255 - r;
            point.x = cloud1->at(i).x;
            point.y = cloud1->at(i).y;
            point.z = cloud1->at(i).z;
            point.r = r;
            point.g = 0;  // 中间色为0，只显示红蓝变化
            point.b = b;
        }
    }

    // 由RGB点云生成cpointcloud对象，并存入entitylist
    auto cloudEntity = m_pMainWin->getPointCloudListMgr()->CreateCompareCloud(*comparisonCloud);
    cloudEntity->isComparsionCloud = true;
    m_pMainWin->getPWinToolWidget()->addToList(cloudEntity);
    m_pMainWin->NotifySubscribe();

    //得到时间编号
    QString TimeString=m_pMainWin->getPWinToolWidget()->getTimeString();
    //得到存放对比图片的路径
    QString CompareImagePath=m_pMainWin->getPWinToolWidget()->getCompareImagePath();

    //总路径
    QString path=CompareImagePath+TimeString+".png";
    qDebug()<<path;
    //保存
    m_pMainWin->getPWinToolWidget()->SaveImage(path);
    //存储在toolwidget中以便在输出报告中使用
    QVector<QString>& PathList=m_pMainWin->getPWinToolWidget()->getImagePaths();
    PathList.append(path);

    ShowColorBar(minDistance, maxDistance);

}

//FPFH+ICP
void VtkWidget::onAlign()
{
    auto& entityList = m_pMainWin->m_EntityListMgr->getEntityList();
    QVector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;

    // 遍历选中的点云实体
    for (int i = 0; i < entityList.size(); i++) {
        CEntity* entity = entityList[i];
        if (!entity->IsSelected()) continue;
        if (entity->GetUniqueType() == enPointCloud) {
            auto& temp = ((CPointCloud*)entity)->m_pointCloud;
            clouds.append(temp.makeShared());
            //删除选中点云

        }
    }

    // 必须有两个点云才能进行配准
    if (clouds.size() != 2) {
        QString message = "点云数目异常(只允许两个点云数据)";
        QMessageBox msgBox;
        msgBox.setWindowTitle("错误");
        msgBox.setText(message);
        msgBox.setIcon(QMessageBox::Critical);
        msgBox.setStandardButtons(QMessageBox::Ok);
        msgBox.exec();
        return;
    }

    pcl::copyPointCloud( *clouds[0], *cloud1);
    pcl::copyPointCloud( *clouds[1], *cloud2);

    for(int i = 0;i < 2;i++)
        // 检查点云是否为空
        if (cloud1->empty() || cloud2->empty()) {
            QMessageBox::warning(this, "Warning", "其中一个或两个点云为空!");
            return;
        }

    // 下采样：提高效率
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampledCloud1(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampledCloud2(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
    voxelGrid.setLeafSize(0.05f, 0.05f, 0.05f);  // 设置叶子大小为 5cm
    voxelGrid.setInputCloud(cloud1);
    voxelGrid.filter(*downsampledCloud1);
    voxelGrid.setInputCloud(cloud2);
    voxelGrid.filter(*downsampledCloud2);

    // FPFH特征提取
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh1(new pcl::PointCloud<pcl::FPFHSignature33>());
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfh2(new pcl::PointCloud<pcl::FPFHSignature33>());

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
    fpfh.setSearchMethod(tree);

    // 计算法线
    pcl::PointCloud<pcl::Normal>::Ptr normals1(new pcl::PointCloud<pcl::Normal>());
    pcl::PointCloud<pcl::Normal>::Ptr normals2(new pcl::PointCloud<pcl::Normal>());
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(0.05);  // 设置法线估计的半径

    ne.setInputCloud(downsampledCloud1);
    ne.compute(*normals1);
    ne.setInputCloud(downsampledCloud2);
    ne.compute(*normals2);

    // 计算FPFH特征
    fpfh.setRadiusSearch(0.1);  // 设置特征计算的半径
    fpfh.setInputCloud(downsampledCloud1);
    fpfh.setInputNormals(normals1);
    fpfh.compute(*fpfh1);

    fpfh.setInputCloud(downsampledCloud2);
    fpfh.setInputNormals(normals2);
    fpfh.compute(*fpfh2);

    // SAC-IA粗配准
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia;
    sac_ia.setInputSource(downsampledCloud1);
    sac_ia.setInputTarget(downsampledCloud2);
    sac_ia.setSourceFeatures(fpfh1);
    sac_ia.setTargetFeatures(fpfh2);
    sac_ia.setMaximumIterations(500);  // 设置最大迭代次数
    sac_ia.setMinSampleDistance(0.05);  // 设置最小采样距离
    sac_ia.setMaxCorrespondenceDistance(0.1);  // 设置最大对应点距离

    pcl::PointCloud<pcl::PointXYZ> sacAlignedCloud;
    sac_ia.align(sacAlignedCloud);  // 粗配准

    if (!sac_ia.hasConverged()) {
        QMessageBox::critical(this, "Error", "FPFH粗配准未收敛！");
        return;
    }

    auto initialTransformation = std::make_shared<Eigen::Matrix4f>(sac_ia.getFinalTransformation());

    // ICP精配准
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(downsampledCloud1);
    icp.setInputTarget(downsampledCloud2);
    icp.setMaximumIterations(50);  // 设置最大迭代次数
    icp.setTransformationEpsilon(1e-8);  // 设置变换容差
    icp.setMaxCorrespondenceDistance(0.05);  // 设置最大对应点距离

    pcl::PointCloud<pcl::PointXYZ> icpFinalCloud;
    icp.align(icpFinalCloud, *initialTransformation);  // 使用FPFH的变换矩阵作为初始对齐

    if (!icp.hasConverged()) {
        // Retry with adjusted parameters
        icp.setMaxCorrespondenceDistance(0.1);  // 增加最大对应点距离
        icp.setMaximumIterations(100);  // 增加最大迭代次数
        icp.align(icpFinalCloud, *initialTransformation);

        if (!icp.hasConverged()) {
            QMessageBox::critical(this, "Error", "ICP 配准未收敛！");
            return;
        }
    }

    if (icp.hasConverged()) {
        pcl::copyPointCloud(icpFinalCloud, *alignedCloud);

        // 显示对齐后的点云
        auto cloudEntity = m_pMainWin->getPointCloudListMgr()->CreateAlignCloud(*alignedCloud);
        m_pMainWin->getPWinToolWidget()->addToList(cloudEntity);
        m_pMainWin->NotifySubscribe();

        // 输出 RMSE（均方根误差）
        double rmse = icp.getFitnessScore();
        QMessageBox::information(this, "Alignment Result", QString("Fine alignment RMSE: %1").arg(rmse));
    } else {
        QMessageBox::critical(this, "Error", "ICP 配准未收敛！");
    }
}

