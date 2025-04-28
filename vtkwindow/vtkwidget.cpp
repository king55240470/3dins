#include "vtkwindow/vtkwidget.h"
#include "vtkwindow/vtkpresetwidget.h"
#include "component/filemanagerwidget.h"
#include <QFileDialog>
#include <QOpenGLContext>
#include <qopenglfunctions.h>
#include <QMessageBox>
#include <vtkInteractorStyle.h>
#include <vtkEventQtSlotConnect.h>
#include <vtkTimerLog.h>
#include <vtkImageResize.h>
#include <vtkCellArray.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_smoothing_laplacian.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/normal_refinement.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/surface/poisson.h>
#include <pcl/surface/impl/poisson.hpp>
#include <pcl/filters/random_sample.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <fbxsdk.h>

VtkWidget::VtkWidget(QWidget *parent)
    : QWidget(parent),
    cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>()),
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
    setUpVtk(mainlayout); // 配置vtk窗口XX
    this->setLayout(mainlayout);
}

void VtkWidget::setUpVtk(QVBoxLayout *layout){
    vtkMenu = new QMenu(m_pMainWin); // 创建菜单
    auto clearAction = new QAction("清除文本标注");
    vtkMenu->addAction(clearAction);
    connect(clearAction, &QAction::triggered, this, [&](){
        this->closeText();
    });

    // 初始化渲染器和交互器
    renderer = vtkSmartPointer<vtkRenderer>::New();
    renderer->SetBackground(0.2, 0.3, 0.5);
    renderer->SetBackground(1, 1, 1);
    renderer->SetGradientBackground(true);
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
        camera->GetPosition();
        camera->GetViewAngle();

        renderer->ResetCamera();
        renWin->Render();
    }
    getRenderWindow()->Render();
}

void VtkWidget::OnMouseMove()
{
    if (!isDragging && !isMiddleDragging) {
        return; // 如果没有拖动且没有按住鼠标中键，则直接返回
    }

    int clickPos[2];
    renWin->GetInteractor()->GetEventPosition(clickPos);

    if(isDragging){
        // 更新当前拖动的文本演员
        infoTextActor->SetPosition(clickPos[0] - 30, clickPos[1] - 20);
        position = infoTextActor->GetPosition();

        // 更新对应的文本框、标题行和叉号的位置
        rectangleActor->SetPosition(position[0] - 5, position[1] - 5);
        titleTextActor->SetPosition(position[0], position[1] + textHeight);
        iconActor->SetPosition(position[0] + textWidth - 5, position[1] + textHeight + 15);

        // 更新对应的指向线段
        Linechange();
    }
    // if(isMiddleDragging){
    //     // 更新所有线段的终点
    //     for (auto it = entityToTextActors.begin(); it != entityToTextActors.end(); ++it)
    //     {
    //         updateEndPoint(it.key());
    //     }
    // }

    // 限制渲染频率
    static double lastRenderTime = vtkTimerLog::GetUniversalTime();
    double currentTime = vtkTimerLog::GetUniversalTime();
    if (currentTime - lastRenderTime > 0.02) {
        getRenderWindow()->Render();
        lastRenderTime = currentTime;
    }
}

void VtkWidget::OnLeftButtonPress()
{
    vtkMenu->hideTearOffMenu(); // 关闭右键菜单栏
    int* clickPos = renWin->GetInteractor()->GetEventPosition();

    // 遍历所有文本框，检查点击位置是否在某个文本框内
    for (auto it = entityToTextActors.begin(); it != entityToTextActors.end(); ++it)
    {
        // 更新文本和标题演员
        infoTextActor = it.value();
        titleTextActor = entityToTitleTextActors[it.key()];

        double bbox[4];
        infoTextActor->GetBoundingBox(renderer, bbox);
        textWidth = bbox[1] - bbox[0];
        textHeight = bbox[3] - bbox[2];

        double width = textWidth + 20; // 加上边距
        double height = textHeight + 10; // 加上边距
        position = infoTextActor->GetPosition();
        double* titlePosition = titleTextActor->GetPosition();
        double titleSize[2];
        titleTextActor->GetSize(renderer, titleSize);

        // 检查点击位置是否在文本框右上角的特定区域内
        double closeBoxSize =  20; // 右上角关闭区域的大小，和叉号尺寸一致
        if (clickPos[0] >= titlePosition[0] + width - closeBoxSize &&
            clickPos[0] <= titlePosition[0] + width &&
            clickPos[1] >= titlePosition[1] + titleSize[1] - closeBoxSize &&
            clickPos[1] <= titlePosition[1] + titleSize[1])
        {            
            closeTextActor(it.key()); // 关闭文本框
            return;
        }

        if (clickPos[0] >= position[0] && clickPos[0] <= position[0] + width &&
            clickPos[1] >= position[1] && clickPos[1] <= position[1] + height)
        {
            // 设置当前拖动的所有actor
            rectangleActor = entityToTextBoxs[it.key()];
            iconActor = entityToIcons[it.key()];
            lineActor = entityToLines[it.key()];
            titleTextActor = entityToTitleTextActors[it.key()];

            isDragging = true; // 开启拖动状态
            renWin->GetInteractor()->SetEventInformation(clickPos[0], clickPos[1], 0, 0, 0, 0);
            renWin->GetInteractor()->SetInteractorStyle(0); // 取消高亮事件
            return;
        }
    }
}

void VtkWidget::OnLeftButtonRelease()
{
    isDragging = false; // 关闭拖动状态
    renWin->GetInteractor()->SetInteractorStyle(m_highlightstyle);
}

void VtkWidget::OnMiddleButtonPress()
{
    isMiddleDragging = true; // 开启中键拖动状态
}

void VtkWidget::OnMiddleButtonRelease()
{
    isMiddleDragging = false;
}

void VtkWidget::setCentity(CEntity* entity)
{
    if (!entity) return;

    // 如果已经存在该图形的信息，直接返回
    if (entityToTextActors.contains(entity)) return;

    createText(entity);
}

vtkSmartPointer<vtkTextActor> &VtkWidget::getInfoText()
{
    return infoTextActor;
}

void VtkWidget::createText(CEntity* entity)
{
    QString qstr = entity->getCEntityInfo();
    QString firstLine = qstr.section('\n', 0, 0); // 提取第一行
    QString remainingText = qstr.section('\n', 1); // 去掉第一行后的文本

    infoTextActor = vtkSmartPointer<vtkTextActor>::New();
    infoTextActor->SetInput(remainingText.toUtf8().constData()); // 设置去掉第一行后的文本
    infoTextActor->GetTextProperty()->SetFontSize(16);
    infoTextActor->GetTextProperty()->SetFontFamilyToTimes();
    infoTextActor->GetTextProperty()->SetColor(MainWindow::InfoTextColor);
    infoTextActor->GetTextProperty()->SetJustificationToLeft();
    infoTextActor->GetTextProperty()->SetBold(1);
    infoTextActor->SetLayerNumber(1);

    // 创建标题文本演员
    titleTextActor = vtkSmartPointer<vtkTextActor>::New();
    titleTextActor->GetTextProperty()->SetFontSize(18); // 设置字体大小
    titleTextActor->GetTextProperty()->SetFontFamilyToTimes(); // 设置字体样式
    titleTextActor->GetTextProperty()->SetColor(1, 1, 0);
    titleTextActor->GetTextProperty()->SetBold(1); // 设置加粗
    titleTextActor->SetInput(firstLine.toUtf8().constData()); // 设置输入文本

    // 计算两个actor大小
    double infoBox[4];
    infoTextActor->GetBoundingBox(renderer, infoBox);
    textWidth = infoBox[1] - infoBox[0];
    textHeight = infoBox[3] - infoBox[2];

    double titleBox[4];
    titleTextActor->GetBoundingBox(renderer, titleBox);
    titleWidth = titleBox[1] - titleBox[0];
    titleHeight = titleBox[3] - titleBox[2];
    double width = titleWidth > textWidth ? titleWidth:textWidth + 30;
    double height = textHeight + titleHeight + 20;

    // 计算文本框的位置
    double x = renWin->GetSize()[0] - width - increaseDis[0];
    double y = renWin->GetSize()[1] - height - increaseDis[1];
    infoTextActor->SetPosition(x, y);
    if(increaseDis[1]  <= renWin->GetSize()[1] - 300){
        increaseDis[1] += 200;
    }
    else {
        increaseDis[1] = 0;
        increaseDis[0] += renWin->GetSize()[0] - 300; // 放到窗口左边显示
    }

    // 检查是否与其他文本框重叠
    for (auto it = entityToTextActors.begin(); it != entityToTextActors.end(); ++it)
    {
        double otherBbox[4];
        it.value()->GetBoundingBox(renderer, otherBbox);
        if (x + width > otherBbox[0] && x < otherBbox[1] &&
            y + height > otherBbox[2] && y < otherBbox[3])
        {
            // 如果重叠，调整位置
            y -= (height + 10);
        }
    }

    titleTextActor->SetPosition(x, y + textHeight); // 设置标题文本的位置

    // 创建文本框
    vtkSmartPointer<vtkActor2D> textBox = createTextBox(infoTextActor, x, y);

    // 创建指向线段
    vtkSmartPointer<vtkActor2D> line = createLine(entity, infoTextActor);

    // 创建叉号
    vtkSmartPointer<vtkActor2D> closeIcon = createCloseIcon(infoTextActor, x, y);

    // 存储信息
    entityToTextActors[entity] = infoTextActor;
    entityToTextBoxs[entity] = textBox;
    entityToLines[entity] = line;
    entityToTitleTextActors[entity] = titleTextActor;
    entityToIcons[entity] = iconActor;

    // 添加到渲染器
    renderer->AddActor(infoTextActor);
    renderer->AddActor(textBox);
    renderer->AddActor(line);
    renderer->AddActor(titleTextActor);
    renderer->AddActor(closeIcon);

    // 绑定鼠标事件
    renWin->GetInteractor()->AddObserver(vtkCommand::LeftButtonPressEvent, this, &VtkWidget::OnLeftButtonPress);
    renWin->GetInteractor()->AddObserver(vtkCommand::MouseMoveEvent, this, &VtkWidget::OnMouseMove);
    renWin->GetInteractor()->AddObserver(vtkCommand::LeftButtonReleaseEvent, this, &VtkWidget::OnLeftButtonRelease);
    renWin->GetInteractor()->AddObserver(vtkCommand::RightButtonPressEvent, this, &VtkWidget::OnRightButtonPress);
    renWin->GetInteractor()->AddObserver(vtkCommand::MiddleButtonPressEvent, this, &VtkWidget::OnMiddleButtonPress);
    renWin->GetInteractor()->AddObserver(vtkCommand::MiddleButtonReleaseEvent, this, &VtkWidget::OnMiddleButtonRelease);

    renWin->Render();
}

// 创建文本框
vtkSmartPointer<vtkActor2D> VtkWidget::createTextBox(vtkSmartPointer<vtkTextActor> textActor, double x, double y)
{
    points = vtkSmartPointer<vtkPoints>::New();
    double bbox[4];
    textActor->GetBoundingBox(renderer, bbox);
    textWidth = bbox[1] - bbox[0];
    textHeight = bbox[3] - bbox[2];
    double width = textWidth + 25;
    double height = textHeight + 40; // 加上标题行的高度

    points->InsertNextPoint(0, 0, 0);
    points->InsertNextPoint(width, 0, 0);
    points->InsertNextPoint(width, height, 0);
    points->InsertNextPoint(0, height, 0);

    lines = vtkSmartPointer<vtkCellArray>::New();
    vtkIdType lineIds[2];
    lineIds[0] = 0; lineIds[1] = 1;
    lines->InsertNextCell(2, lineIds);
    lineIds[0] = 1; lineIds[1] = 2;
    lines->InsertNextCell(2, lineIds);
    lineIds[0] = 2; lineIds[1] = 3;
    lines->InsertNextCell(2, lineIds);
    lineIds[0] = 3; lineIds[1] = 0;
    lines->InsertNextCell(2, lineIds);

    polygons = vtkSmartPointer<vtkCellArray>::New();
    vtkIdType ids[4] = {0, 1, 2, 3};
    polygons->InsertNextCell(4, ids);

    rectangle = vtkSmartPointer<vtkPolyData>::New();
    rectangle->SetPoints(points);
    rectangle->SetLines(lines);
    rectangle->SetPolys(polygons);

    rectangleMapper = vtkSmartPointer<vtkPolyDataMapper2D>::New();
    rectangleMapper->SetInputData(rectangle);

    rectangleActor = vtkSmartPointer<vtkActor2D>::New();
    rectangleActor->SetMapper(rectangleMapper);
    rectangleActor->GetProperty()->SetColor(0.6902, 0.8784, 0.9020);
    rectangleActor->GetProperty()->SetOpacity(0.15);
    rectangleActor->GetProperty()->SetLineWidth(4);
    rectangleActor->SetPosition(x - 5, y - 5);
    rectangleActor->SetLayerNumber(3);
    getRenderer();

    return rectangleActor;
}

vtkSmartPointer<vtkActor2D> VtkWidget::createLine(CEntity* entity, vtkSmartPointer<vtkTextActor> textActor)
{
    double* a = textActor->GetPosition();
    CPosition b;
    QString filename;

    if (entity->GetUniqueType() == enDistance)
    {
        CDistance* dis = static_cast<CDistance*>(entity); // 安全类型转换
        CPosition begin = dis->getbegin();
        b.x = begin.x;
        b.y = begin.y;
        b.z = begin.z;
    }
    else if (entity->GetUniqueType() == enPoint)
    {
        CPoint* point = static_cast<CPoint*>(entity);
        b = point->GetPt();
    }
    else if (entity->GetUniqueType() == enLine)
    {
        CLine* line = static_cast<CLine*>(entity);
        b = line->getBegin();
    }
    else if (entity->GetUniqueType() == enCircle)
    {
        CCircle* circle = static_cast<CCircle*>(entity);
        b = circle->getCenter();
    }
    else if (entity->GetUniqueType() == enSphere)
    {
        CSphere* s = static_cast<CSphere*>(entity);
        b = s->getCenter();
    }
    else if (entity->GetUniqueType() == enPlane)
    {
        CPlane* s = static_cast<CPlane*>(entity);
        b = s->getCenter();
    }
    else if (entity->GetUniqueType() == enCylinder)
    {
        CCylinder* s = static_cast<CCylinder*>(entity);
        b = s->getBtm_center();
    }
    else if (entity->GetUniqueType() == enCone)
    {
        CCone* s = static_cast<CCone*>(entity);
        b = s->getVertex();
    }
    else if(entity->GetUniqueType() == enAngle){
        CAngle* angle = static_cast<CAngle*>(entity);
        b = angle->getVertex(); // 指向线段终点是两条线的 "交点"
    }
    else if(entity->GetUniqueType() == enPointCloud){
        CPointCloud* cloud = static_cast<CPointCloud*>(entity);
        auto point = cloud->m_pointCloud.points[0];
        b = CPosition(point.x, point.y, point.z);
    }
    else if(entity->GetUniqueType() == enSurfaces){
        CSurfaces* surface = static_cast<CSurfaces*>(entity);
        auto point = surface->m_pointCloud.points[0];
        b = CPosition(point.x, point.y, point.z);
    }
    entityToEndPoints[entity] = b;

    coordinate = vtkSmartPointer<vtkCoordinate>::New();
    coordinate->SetValue(b.x, b.y, b.z);
    coordinate->SetCoordinateSystemToWorld();
    int* viewportMidPoint = coordinate->GetComputedViewportValue(renderer);

    points = vtkSmartPointer<vtkPoints>::New();
    points->InsertNextPoint(a[0], a[1], 0.0);
    points->InsertNextPoint(viewportMidPoint[0], viewportMidPoint[1], viewportMidPoint[2]);

    linePolyData = vtkSmartPointer<vtkPolyData>::New();
    linePolyData->SetPoints(points);

    lines = vtkSmartPointer<vtkCellArray>::New();
    lines->InsertNextCell(2);
    lines->InsertCellPoint(0);
    lines->InsertCellPoint(1);
    linePolyData->SetLines(lines);

    lineMapper = vtkSmartPointer<vtkPolyDataMapper2D>::New();
    lineMapper->SetInputData(linePolyData);

    lineActor = vtkSmartPointer<vtkActor2D>::New();
    lineActor->SetMapper(lineMapper);
    lineActor->GetProperty()->SetColor(1, 1, 0);
    lineActor->GetProperty()->SetLineWidth(2);

    return lineActor;
}

vtkSmartPointer<vtkActor2D> VtkWidget::createCloseIcon(vtkSmartPointer<vtkTextActor> textActor, double x, double y)
{
    double bbox[4];
    textActor->GetBoundingBox(renderer, bbox);
    double textWidth = bbox[1] - bbox[0];
    double textHeight = bbox[3] - bbox[2];
    double width = textWidth + 20;
    double height = textHeight + 40; // 加上标题行的高度

    // 创建点,用于绘制叉号
    crossPoints = vtkSmartPointer<vtkPoints>::New();
    crossPoints->InsertNextPoint(0, 0, 0); // 左下角
    crossPoints->InsertNextPoint(20, 20, 0);  // 右上角
    crossPoints->InsertNextPoint(0, 20, 0); // 左上角
    crossPoints->InsertNextPoint(20, 0, 0); // 右下角

    // 创建线
    crossLines = vtkSmartPointer<vtkCellArray>::New();
    vtkSmartPointer<vtkLine> line1 = vtkSmartPointer<vtkLine>::New();
    line1->GetPointIds()->SetId(0, 0); // 左下角
    line1->GetPointIds()->SetId(1, 1); // 右上角
    vtkSmartPointer<vtkLine> line2 = vtkSmartPointer<vtkLine>::New();
    line2->GetPointIds()->SetId(0, 2); // 左上角
    line2->GetPointIds()->SetId(1, 3); // 右下角
    crossLines->InsertNextCell(line1);
    crossLines->InsertNextCell(line2);

    // 创建多边形数据
    crossPolyData = vtkSmartPointer<vtkPolyData>::New();
    crossPolyData->SetPoints(crossPoints);
    crossPolyData->SetLines(crossLines);

    // 创建映射器
    crossMapper = vtkSmartPointer<vtkPolyDataMapper2D>::New();
    crossMapper->SetInputData(crossPolyData);

    // 创建叉号演员
    iconActor = vtkSmartPointer<vtkActor2D>::New();
    iconActor->SetMapper(crossMapper);
    iconActor->GetProperty()->SetLineWidth(4);
    iconActor->GetProperty()->SetColor(MainWindow::HighLightColor);
    iconActor->SetPosition(x + width - 25, y + height - 25);

    return iconActor;
}

void VtkWidget::Linechange()
{
    if (!isDragging || !infoTextActor || !lineActor) {
        return; // 如果没有拖动或相关对象为空，则直接返回
    }

    // 获取当前拖动的文本框位置
    double* textPosition = infoTextActor->GetPosition();

    // 获取指向线段的终点位置（从实体到终点映射中获取）
    endPoint = entityToEndPoints[getEntityFromTextActor(infoTextActor)];

    // 将终点从世界坐标转换为视口坐标
    coordinate = vtkSmartPointer<vtkCoordinate>::New();
    coordinate->SetValue(endPoint.x, endPoint.y, endPoint.z);
    coordinate->SetCoordinateSystemToWorld();
    int* viewportMidPoint = coordinate->GetComputedViewportValue(renderer);

    // 更新指向线段的起点和终点
    points = vtkSmartPointer<vtkPoints>::New();
    points->InsertNextPoint(textPosition[0], textPosition[1], 0.0); // 起点为文本框位置
    points->InsertNextPoint(viewportMidPoint[0], viewportMidPoint[1], viewportMidPoint[2]); // 终点为视口坐标

    // 更新线段的映射数据
    linePolyData->SetPoints(points);
    lineMapper->SetInputData(linePolyData);
    lineActor->SetMapper(lineMapper);
}

void VtkWidget::updateEndPoint(CEntity *entity)
{
    // 获取指向线段的终点
    CPosition end = entityToEndPoints[getEntityFromTextActor(infoTextActor)];

    // 将鼠标位置转换为世界坐标
    coordinate = vtkSmartPointer<vtkCoordinate>::New();
    coordinate->SetValue(end.x, end.y, end.z);
    coordinate->SetCoordinateSystemToViewport();
    double* worldPos = coordinate->GetComputedWorldValue(renderer);

    // 更新终点位置
    entityToEndPoints[entity].x = worldPos[0];
    entityToEndPoints[entity].y = worldPos[1];
    entityToEndPoints[entity].z = worldPos[2];

    // 更新指向线段
    Linechange();
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
    for (auto it = entityToTitleTextActors.begin(); it != entityToTitleTextActors.end(); ++it)
    {
        renderer->RemoveActor(it.value());
    }
    entityToTextActors.clear();
    entityToTextBoxs.clear();
    entityToLines.clear();
    entityToIcons.clear();
    entityToTitleTextActors.clear();
    // 重置所有文本所占位置
    increaseDis[0] = 0;
    increaseDis[1] = 0;

    vtkMenu->hideTearOffMenu(); // 关闭右键菜单栏
    renWin->Render();
}
// 删除选中的文本框
void VtkWidget::closeTextActor(CEntity* entity)
{
    if (entityToTextActors.contains(entity))
    {
        // 移除文本
        if(entityToTextActors[entity]){
            renderer->RemoveActor(entityToTextActors[entity]);
        }
        // 移除指向线段
        if(entityToLines[entity]){
            renderer->RemoveActor(entityToLines[entity]);
        }
        // 移除图标
        if (entityToIcons.contains(entity))
        {
            renderer->RemoveActor(entityToIcons[entity]);
        }
        // 移除背景矩形框
        if (entityToTextBoxs.contains(entity))
        {
            renderer->RemoveActor(entityToTextBoxs[entity]);
        }
        // 移除标题文本演员
        if (entityToTitleTextActors.contains(entity))
        {
            renderer->RemoveActor(entityToTitleTextActors[entity]);
        }

        // 从映射中移除
        entityToTextActors.remove(entity);
        entityToTextBoxs.remove(entity);
        entityToLines.remove(entity);
        entityToIcons.remove(entity);
        entityToTitleTextActors.remove(entity);
        entityToEndPoints.remove(entity);

        // 去掉该文本框占用的位置
        if (increaseDis[1] >= 200) {
            increaseDis[1] -= 200;
        } else {
            increaseDis[1] = 0;
        }
        // 重新渲染窗口
        renWin->Render();
    }
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
    // 保存色温尺颜色信息
    vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
    colors->SetNumberOfComponents(3); // RGB

    // 定义色温尺的起点和终点
    int barHeight = 200; // 色温尺的高度
    int barWidth = 30; // 色温尺的宽度
    auto Width = renWin->GetSize()[0];
    auto Height = renWin->GetSize()[1];

    // 为每个顶点设置颜色
    colors->InsertTuple3(0, 255, 0, 0); // 右上角
    colors->InsertTuple3(1, 255, 0, 0); // 左上角
    colors->InsertTuple3(2, 0, 0, 255); // 右下角
    colors->InsertTuple3(3, 0, 0, 255); // 左下角

    // 将颜色数据绑定到线段上（通过绘制多边形来模拟颜色条）
    vtkSmartPointer<vtkPolyData> colorBarPolyData = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkPoints> colorBarPoints = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkCellArray> colorBarPolys = vtkSmartPointer<vtkCellArray>::New();

    // 在窗口右上角插入四个点
    colorBarPoints->InsertNextPoint(Width - barWidth - 20, Height - 20, 0);
    colorBarPoints->InsertNextPoint(Width - 20, Height - 20, 0);
    colorBarPoints->InsertNextPoint(Width - 20, Height - barHeight - 20, 0);
    colorBarPoints->InsertNextPoint(Width - barWidth - 20, Height - barHeight - 20, 0);

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

    vtkSmartPointer<vtkTextMapper> maxTextMapper = vtkSmartPointer<vtkTextMapper>::New();
    maxTextMapper->SetInput(std::to_string(maxDistance).c_str());
    vtkSmartPointer<vtkActor2D> maxTextActor = vtkSmartPointer<vtkActor2D>::New();
    maxTextActor->SetMapper(maxTextMapper);

    // 调整两个文本标注的位置以适应显示
    maxTextActor->SetPosition(Width - barWidth - 160, Height - 50);
    minTextActor->SetPosition(Width - barWidth - 160, Height - barHeight - 10);
    minTextMapper->GetTextProperty()->SetFontSize(15);
    maxTextMapper->GetTextProperty()->SetFontSize(15);

    // 获取渲染器并添加色温条和标注
    renderer->AddActor2D(colorBarActor);
    renderer->AddActor2D(minTextActor);
    renderer->AddActor2D(maxTextActor);

    // 刷新渲染窗口
    renderer->Render();
}

void VtkWidget::createScaleBar()
{
    if(scaleText){
        //dreturn;
    }
    double x=renWin->GetSize()[0]-320;
    double y=renWin->GetSize()[1]-170;

    lineSource = vtkSmartPointer<vtkLineSource>::New();
    lineSource->SetPoint1(0, 0, 0);
    lineSource->SetPoint2(200, 0, 0); // 初始长度，后续动态调整

    // ========== 坐标系设置（关键） ==========
    vtkNew<vtkCoordinate> coordinate;
    coordinate->SetCoordinateSystemToDisplay(); // 显示坐标系

    vtkNew<vtkPolyDataMapper2D> mapper;
    mapper->SetInputConnection(lineSource->GetOutputPort());
    mapper->SetTransformCoordinate(coordinate); // 绑定坐标系

    // ========== 标尺线样式 ==========
    scaleBarActor = vtkSmartPointer<vtkActor2D>::New();
    scaleBarActor->SetMapper(mapper);
    scaleBarActor->GetProperty()->SetColor(1, 1, 1); // 红色
    scaleBarActor->GetProperty()->SetLineWidth(3);

    scaleText = vtkSmartPointer<vtkTextActor>::New();
    //scaleText->SetTextScaleModeToViewport();
    scaleText->GetTextProperty()->SetFontSize(15);
    scaleText->GetTextProperty()->SetColor(1.0, 1.0, 0);
    scaleText->SetInput("比例Scale : 1.0 units(mm)");

    scaleText->SetPosition(x+100,y-470); // 视口中的位置

    //scaleBarActor->SetPosition(x+100,y-450);
    qDebug()<<x<<y;
    scaleBarActor->SetLayerNumber(2); // 置于顶层
    scaleText->SetLayerNumber(2);
    renderer->AddActor2D(scaleText);
    renderer->AddActor2D(scaleBarActor);
    UpdateScaleBar();
    //renderer->Render();
}

void VtkWidget::UpdateScaleBar()
{
    if (!renWin || !renderer) return;

    int* winSize = renWin->GetSize();
    if (winSize[0] <= 0 || winSize[1] <= 0) return;

    const int fixedPixelLength = 200;
    const int marginRight = 70;
    const int marginBottom = 70;

    //int lineX = winSize[0] - marginRight - fixedPixelLength;
    //int lineY = marginBottom;

    //scaleBarActor->SetPosition(lineX, lineY);
    //scaleText->SetPosition(lineX + 55, lineY - 20); // 文本位于标尺下方20像素
    vtkCamera* camera = renderer->GetActiveCamera();
    double physicalLength = 0.0;

    if (camera->GetParallelProjection()) {
        // 平行投影精确计算
        double parallelScale = camera->GetParallelScale();
        physicalLength = (fixedPixelLength * 2.0 * parallelScale) / winSize[1];
    } else {
        // 透视投影近似计算
        double viewAngle = vtkMath::RadiansFromDegrees(camera->GetViewAngle());
        double distance = camera->GetDistance();
        physicalLength = (fixedPixelLength * 2.0 * distance * tan(viewAngle/2)) / winSize[1];
    }
    // 获取整洁的物理长度
    double desiredPhysicalLength = roundToNearestNiceValue(physicalLength);

    // 计算所需的像素长度
    double desiredPixelLength;
    if (camera->GetParallelProjection()) {
        double parallelScale = camera->GetParallelScale();
        desiredPixelLength = (desiredPhysicalLength * winSize[1]) / (2.0 * parallelScale);
    } else {
        double viewAngle = vtkMath::RadiansFromDegrees(camera->GetViewAngle());
        double distance = camera->GetDistance();
        desiredPixelLength = (desiredPhysicalLength * winSize[1]) / (2.0 * distance * std::tan(viewAngle / 2));
    }

    // 更新线段长度
    lineSource->SetPoint2(desiredPixelLength, 0, 0);
    lineSource->Modified();

    // 调整标尺位置（右对齐）
    int lineX = winSize[0] - marginRight - desiredPixelLength;
    scaleBarActor->SetPosition(lineX, marginBottom);

    // 更新文本
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2) << desiredPhysicalLength << " mm";
    scaleText->SetInput(ss.str().c_str());

    // 文本居中
    int textX = lineX + desiredPixelLength / 2 - 30; // 调整偏移量
    scaleText->SetPosition(textX, marginBottom - 20);

    //scaleText->GetTextProperty()->SetFontFamilyToArial(); // 默认字体
    //scaleText->GetTextProperty()->SetFontFile("C:/qcon/3dins/ebrima.ttf");

    if (renWin->GetMapped()) {
        renWin->Render();
    }
}

void VtkWidget::attachInteractor()
{
    // ========== 创建通用回调 ==========
    auto callback = vtkSmartPointer<vtkCallbackCommand>::New();
    callback->SetClientData(this);
    callback->SetCallback([](vtkObject*, unsigned long, void* clientData, void*) {
        static_cast<VtkWidget*>(clientData)->UpdateScaleBar();
    });

    // ========== 绑定关键事件 ==========
    // 窗口大小变化
    renWin->AddObserver(vtkCommand::WindowResizeEvent, callback);

    // 相机参数变化
    renderer->GetActiveCamera()->AddObserver(vtkCommand::ModifiedEvent, callback);

    // 用户交互操作
    vtkRenderWindowInteractor* interactor = renWin->GetInteractor();
    if (interactor) {
        interactor->AddObserver(vtkCommand::InteractionEvent, callback);
        interactor->AddObserver(vtkCommand::EndInteractionEvent, callback);
    }
}

double VtkWidget::roundToNearestNiceValue(double value)
{
    if (value <= 0) return 0.0;

    double exponent = std::floor(std::log10(value));
    double normalized = value / std::pow(10, exponent);

    double nice = 1.0;
    if (normalized < 1.5) {
        nice = 1.0;
    } else if (normalized < 2.5) {
        nice = 2.0;
    } else if (normalized < 5.0) {
        nice = 5.0;
    } else {
        nice = 10.0;
    }

    if (nice == 10.0) {
        exponent += 1;
        nice = 1.0;
    }

    return nice * std::pow(10, exponent);
}

void VtkWidget::OnRightButtonPress()
{
    vtkMenu->showTearOffMenu(); // 弹出菜单栏
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
    // 清除所有文本标注的记录
    entityToTextActors.clear();
    entityToTextBoxs.clear();
    entityToLines.clear();
    entityToIcons.clear();

    CPointCloud::getActorToPointCloud().clear();
    // 遍历entitylist绘制图形并加入渲染器
    for(auto i = 0;i < entitylist.size();i++){
        int constructFlag=0;
        int identifyFlag=0;

        // 循环判断是哪种元素
        for(int j=0;j<constructEntityList.size();j++){
            QString key=constructEntityList[j]->GetObjectCName() + "  " + constructEntityList[j]->GetObjectAutoName();
            QString name=entitylist[i]->GetObjectCName()+"  "+entitylist[i]->GetObjectAutoName();
            if(name == key){//是构建的元素
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
                QString name=entitylist[i]->GetObjectCName()+"  "+entitylist[i]->GetObjectAutoName();
                if(name == key){//是构建的元素
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
    createScaleBar(); // 重新创建比例尺
    attachInteractor();
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

void VtkWidget::createActorController()
{
    actorAdjustDialog = new QDialog(this);
    actorAdjustDialog->setWindowTitle("调整图形渲染的粗细");
    actorAdjustDialog->resize(200, 100);

    // 添加图标和样式
    QIcon addIcon(":/style/add.png");
    QIcon subIcon(":/style/sub.png");
    QString buttonStyle("QPushButton { border: none; background-color: transparent; }");

    // 创建点大小调整的控件
    QLabel *pointSizeLabel = new QLabel("点的大小:", actorAdjustDialog);
    QLabel *currentPointSizeLabel = new QLabel(QString::number(MainWindow::ActorPointSize), actorAdjustDialog);
    QPushButton *pointSizeAddBtn = new QPushButton(actorAdjustDialog);
    QPushButton *pointSizeSubBtn = new QPushButton(actorAdjustDialog);
    pointSizeAddBtn->setIcon(addIcon);
    pointSizeAddBtn->setIconSize(QSize(30, 30));
    pointSizeAddBtn->setStyleSheet(buttonStyle);
    pointSizeSubBtn->setIcon(subIcon);
    pointSizeSubBtn->setIconSize(QSize(30, 30));
    pointSizeSubBtn->setStyleSheet(buttonStyle);

    // 创建线宽调整的控件
    QLabel *lineWidthLabel = new QLabel("线条宽度:", actorAdjustDialog);
    QLabel *currentLineWidthLabel = new QLabel(QString::number(MainWindow::ActorLineWidth), actorAdjustDialog);
    QPushButton *lineWidthAddBtn = new QPushButton(actorAdjustDialog);
    QPushButton *lineWidthSubBtn = new QPushButton(actorAdjustDialog);
    lineWidthAddBtn->setIcon(addIcon);
    lineWidthAddBtn->setIconSize(QSize(30, 30));
    lineWidthAddBtn->setStyleSheet(buttonStyle);
    lineWidthSubBtn->setIcon(subIcon);
    lineWidthSubBtn->setIconSize(QSize(30, 30));
    lineWidthSubBtn->setStyleSheet(buttonStyle);

    // 创建布局
    QVBoxLayout *mainLayout = new QVBoxLayout(actorAdjustDialog);

    // 点大小调整布局
    QHBoxLayout *pointSizeLayout = new QHBoxLayout();
    pointSizeLayout->addWidget(pointSizeLabel);
    pointSizeLayout->addWidget(currentPointSizeLabel);
    pointSizeLayout->addWidget(pointSizeSubBtn);
    pointSizeLayout->addWidget(pointSizeAddBtn);

    // 线宽调整布局
    QHBoxLayout *lineWidthLayout = new QHBoxLayout();
    lineWidthLayout->addWidget(lineWidthLabel);
    lineWidthLayout->addWidget(currentLineWidthLabel);
    lineWidthLayout->addWidget(lineWidthSubBtn);
    lineWidthLayout->addWidget(lineWidthAddBtn);

    // 添加到主布局
    mainLayout->addLayout(pointSizeLayout);
    mainLayout->addLayout(lineWidthLayout);

    // 连接信号与槽
    connect(pointSizeAddBtn, &QPushButton::clicked, [=]() {
        if(MainWindow::ActorPointSize < 10)
            MainWindow::ActorPointSize += 1;
        currentPointSizeLabel->setText(QString::number(MainWindow::ActorPointSize));
        reDrawCentity();
    });

    connect(pointSizeSubBtn, &QPushButton::clicked, [=]() {
        if(MainWindow::ActorPointSize > 1)
            MainWindow::ActorPointSize -= 1;
        currentPointSizeLabel->setText(QString::number(MainWindow::ActorPointSize));
        reDrawCentity();
    });

    connect(lineWidthAddBtn, &QPushButton::clicked, [=]() {
        if(MainWindow::ActorLineWidth < 7)
            MainWindow::ActorLineWidth += 1;
        currentLineWidthLabel->setText(QString::number(MainWindow::ActorLineWidth));
        reDrawCentity();
    });

    connect(lineWidthSubBtn, &QPushButton::clicked, [=]() {
        if(MainWindow::ActorLineWidth > 0)
            MainWindow::ActorLineWidth -= 1;
        currentLineWidthLabel->setText(QString::number(MainWindow::ActorLineWidth));
        reDrawCentity();
    });
    actorAdjustDialog->show();
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
        renWin->Render();
    }
}

// 切换相机视角3
void VtkWidget::onFrontView(){
    vtkCamera *camera = renderer->GetActiveCamera();
    camera->GetPosition();
    if (camera) {
        camera->SetPosition(0, -1, 0);  // 重置相机位置为正视
        camera->SetFocalPoint(0, 0, 0);
        camera->SetViewUp(0, 0, 1);
        camera->OrthogonalizeViewUp(); // 确保与SetViewUp方向正交

        // 重新设置相机并渲染
        renderer->ResetCamera();
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
        renWin->Render();
    }
}

void VtkWidget::ExportPointCloudToFBX(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, const std::string& filepath) {
    // 初始化 FBX 管理器
    FbxManager* manager = FbxManager::Create();

    // 设置 IO 设置
    FbxIOSettings* ios = FbxIOSettings::Create(manager, IOSROOT);
    manager->SetIOSettings(ios);

    // 创建场景
    FbxScene* scene = FbxScene::Create(manager, "PointCloudScene");

    // 创建 mesh
    FbxMesh* mesh = FbxMesh::Create(scene, "PointCloudMesh");

    int numVertices = cloud->size();
    mesh->InitControlPoints(numVertices);
    FbxVector4* controlPoints = mesh->GetControlPoints();

    for (int i = 0; i < numVertices; ++i) {
        const auto& pt = cloud->points[i];
        controlPoints[i] = FbxVector4(pt.x, pt.y, pt.z);
    }

    // 添加颜色
    FbxGeometryElementVertexColor* vertexColor = mesh->CreateElementVertexColor();
    vertexColor->SetMappingMode(FbxGeometryElement::eByControlPoint);
    vertexColor->SetReferenceMode(FbxGeometryElement::eDirect);

    for (int i = 0; i < numVertices; ++i) {
        const auto& pt = cloud->points[i];
        vertexColor->GetDirectArray().Add(FbxColor(pt.r / 255.0, pt.g / 255.0, pt.b / 255.0, 1.0));
    }

    // 创建节点
    FbxNode* meshNode = FbxNode::Create(scene, "PointCloudNode");
    meshNode->SetNodeAttribute(mesh);
    scene->GetRootNode()->AddChild(meshNode);

    // 创建导出器
    FbxExporter* exporter = FbxExporter::Create(manager, "");

    if (!exporter->Initialize(filepath.c_str(), -1, manager->GetIOSettings())) {
        printf("Failed to initialize FBX exporter: %s\n", exporter->GetStatus().GetErrorString());
        return;
    }

    // 导出场景
    exporter->Export(scene);
    exporter->Destroy();

    // 清理
    manager->Destroy();
}

double *VtkWidget::getViewAngles()
{
    vtkCamera *camera = renderer->GetActiveCamera();
    if (!camera)    return nullptr;

    // 获取相机的方向向量
    double projDir[3];
    camera->GetDirectionOfProjection(projDir);
    double viewUp[3];
    camera->GetViewUp(viewUp);

    // 计算相机的正交向量
    double right[3];
    vtkMath::Cross(projDir, viewUp, right);

    // 构建旋转矩阵
    double rotationMatrix[3][3];
    for (int i = 0; i < 3; i++) {
        rotationMatrix[0][i] = right[i];
        rotationMatrix[1][i] = viewUp[i];
        rotationMatrix[2][i] = -projDir[i]; // 注意：投影方向是相机的负 z 轴方向
    }

    // 计算欧拉角（以弧度为单位）
    double phi, theta, psi;
    // 根据旋转矩阵计算欧拉角的公式（这里使用 XYZ 固定角序列）
    theta = asin(rotationMatrix[2][0]);
    phi = atan2(rotationMatrix[2][1] / cos(theta), rotationMatrix[2][2] / cos(theta));
    psi = atan2(rotationMatrix[1][0] / cos(theta), rotationMatrix[0][0] / cos(theta));

    // 转换为角度
    phi = vtkMath::DegreesFromRadians(phi);
    theta = vtkMath::DegreesFromRadians(theta);
    psi = vtkMath::DegreesFromRadians(psi);

    // 创建一个 double 数组来存储结果
    double* angles = new double[3];
    angles[0] = phi;   // 绕 X 轴的旋转角度
    angles[1] = theta; // 绕 Y 轴的旋转角度
    angles[2] = psi;   // 绕 Z 轴的旋转角度

    return angles;
}

double *VtkWidget::getBoundboxData(CEntity* entity)
{
    // 得到对应的actor
    auto& actorMap = m_pMainWin->getactorToEntityMap();
    auto actor = actorMap.key(entity);
    double* boxData = new double();

    // 获取 actor 的边界框
    double bounds[6];
    actor->GetBounds(bounds);
    double length = bounds[1] - bounds[0];
    double width = bounds[3] - bounds[2];
    double height = bounds[5] - bounds[4];
    boxData[0] = length;
    boxData[1] = width;
    boxData[2] = height;

    return boxData;
}

CPosition VtkWidget::getBoundboxCenter(CEntity *entity)
{
    // 得到对应的actor
    auto& actorMap = m_pMainWin->getactorToEntityMap();
    auto actor = actorMap.key(entity);

    // 获取 actor 的边界框
    double bounds[6];
    actor->GetBounds(bounds);
    CPosition center(bounds[1] - bounds[0], bounds[3] - bounds[2], bounds[5] - bounds[4]);
    qDebug() << center.x << center.y << center.z;

    return CPosition(center.x/2, center.y/2, center.z);
}

int VtkWidget::adjustMeanK(size_t pointCount)
{
    if(pointCount <= 10000)
        return 20;
    else if(pointCount > 10000 && pointCount <= 100000)
        return 50;
    else if(pointCount > 10000 && pointCount <= 500000)
        return 100;
    else return 150;
}

double VtkWidget::adjustStddevThresh(size_t pointCount)
{
    if(pointCount <= 10000)
        return 3.0;
    else if(pointCount > 10000 && pointCount <= 100000)
        return 2.0;
    else return 1.0;
}

int VtkWidget::FilterCount(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    if(cloud->size() > 1500000) return 4;
    else if(cloud->size() > 1000000 && cloud->size() < 1500000) return 3;
    else if(cloud->size() > 500000 && cloud->size() < 1000000) return 2;
    else if(cloud->size() > 100000 && cloud->size() < 500000) return 1;
    else return 0;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr VtkWidget::onFilter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    if(!cloud){
        m_pMainWin->getPWinVtkPresetWidget()->setWidget("滤波的输入点云为空！");
        return nullptr;
    }

    // 统计滤波去噪
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    int kMean = adjustMeanK(cloud->size());
    double thresh = adjustStddevThresh(cloud->size());
    sor.setInputCloud(cloud);
    sor.setMeanK(kMean);   // 考虑的邻近点的数量
    sor.setStddevMulThresh(thresh);  // 判断离群点的阈值
    sor.filter(*cloud_filtered);

    return cloud_filtered;
}

void VtkWidget::onFilter()
{
    auto entityList = m_pMainWin->m_EntityListMgr->getEntityList();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);

    for(int i=0;i<entityList.size();i++){
        CEntity* entity=entityList[i];
        if(entity->GetUniqueType()==enPointCloud && entity->IsSelected()){
            auto& temp=((CPointCloud*)entity)->m_pointCloud;
            cloud = temp.makeShared();
            break;
        }
    }

    if(cloud->empty()){
        m_pMainWin->getPWinVtkPresetWidget()->setWidget(QString("点云为空！"));
        return;
    }

    int cnt = FilterCount(cloud);
    for(int i = 0;i < cnt;i++){
        cloud_filtered = onFilter(cloud);
        cloud = cloud_filtered;
    }

    auto cloudEntity = m_pMainWin->getPointCloudListMgr()->CreateFilterCloud(*cloud_filtered);
    m_pMainWin->getPWinToolWidget()->addToList(cloudEntity);
    m_pMainWin->NotifySubscribe();
}

// 比较两个点云的处理函数
void VtkWidget::onCompare()
{
    auto entityList = m_pMainWin->m_EntityListMgr->getEntityList();
    auto cloudptr= m_pMainWin->getpWinFileMgr()->cloudptr;
    auto marksMap = m_pMainWin->getpWinFileMgr()->getContentItemMap();
    QString logInfo; // 用于在右下角输出调用日志
    bool isCut=false;

    QVector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;
    QVector<CObject*>parentlist;
    for(int i=0;i<entityList.size();i++){
        CEntity* entity=entityList[i];
        if(!entity->IsSelected())continue;
        if(entity->GetUniqueType()==enPointCloud){
            CPointCloud* cloud=(CPointCloud*)entity;
            if(cloud->isCut)isCut=true;
            auto & temp=((CPointCloud*)entity)->m_pointCloud;
            parentlist.append(entity);
            clouds.append(temp.makeShared());
            logInfo += ((CPointCloud*)entity)->m_strAutoName + ' '; //添加对比的点云编号
        }
    }

    if(clouds.size()!=2){
        m_pMainWin->getPWinVtkPresetWidget()->setWidget("需要两个点云！");
        return ;
    }

    pcl::copyPointCloud( *clouds[0], *cloud1);
    pcl::copyPointCloud( *clouds[1], *cloud2);

    // 检查点云是否为空
    if (cloud1->empty() || cloud2->empty()) {
        m_pMainWin->getPWinVtkPresetWidget()->setWidget("点云为空！");
        return;
    }

    // 创建KD-Tree用于点云2
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud1);

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

    // 遍历点云2中的每个点，找到与点云1中最近点的距离
    for (size_t i = 0; i < cloud2->size(); ++i) {
        if (kdtree.nearestKSearch(cloud2->at(i), 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
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
            point.x = cloud2->at(i).x;
            point.y = cloud2->at(i).y;
            point.z = cloud2->at(i).z;
            point.r = r;
            point.g = 0;  // 中间色为0，只显示红蓝变化
            point.b = b;
        }
    }

    // 由RGB点云生成cpointcloud对象，并存入entitylist
    auto cloudEntity = m_pMainWin->getPointCloudListMgr()->CreateCompareCloud(*comparisonCloud);
    cloudEntity->parent=parentlist;
    m_pMainWin->getPWinToolWidget()->addToList(cloudEntity);
    m_pMainWin->NotifySubscribe();

    m_pMainWin->getPWinFileManagerWidget()->allHide("对比");

    // 导出为ply文件
    pcl::PLYWriter writer;
    writer.write("D:/testFiles/compareCloud.ply", *comparisonCloud, true); // true = 写入ASCII格式（false 为二进制）

    // 导出为fbx文件
    ExportPointCloudToFBX(comparisonCloud,"D:/outout.fbx");

    //调用保存图像函数
    m_pMainWin->getPWinToolWidget()->onSaveImage();
    QString path_front=m_pMainWin->getPWinToolWidget()->getlastCreatedImageFileFront();
    QString path_top=m_pMainWin->getPWinToolWidget()->getlastCreatedImageFileTop();
    QString path_right=m_pMainWin->getPWinToolWidget()->getlastCreatedImageFileRight();

    m_pMainWin->getPWinFileManagerWidget()->allRecover();

    if(isCut){
        //存储局部对比点云图像的路径
        QVector<QString>& PathList=m_pMainWin->getPWinToolWidget()->getImagePaths_part();
        PathList.append( path_front);
        PathList.append( path_top);
        PathList.append( path_right);
    }else{
        //存储全局对比点云图像的路径
        QVector<QString>& PathList=m_pMainWin->getPWinToolWidget()->getImagePaths();
        PathList.append( path_front);
        PathList.append( path_top);
        PathList.append( path_right);
    }

    ShowColorBar(minDistance, maxDistance);
    // 添加日志输出
    logInfo += "对比完成";
    m_pMainWin->getPWinVtkPresetWidget()->setWidget(logInfo);
}

//FPFH+ICP
void VtkWidget::onAlign()
{
    auto& entityList = m_pMainWin->m_EntityListMgr->getEntityList();
    QVector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;
    QString logInfo;

    // 收集选中的点云（确保不修改原始实体）
    for (int i = 0; i < entityList.size(); i++) {
        CEntity* entity = entityList[i];
        if (!entity->IsSelected()) continue;
        if (entity->GetUniqueType() == enPointCloud) {
            // 获取点云的共享指针，确保原数据不被释放
            auto pcEntity = static_cast<CPointCloud*>(entity);
            clouds.append(pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(pcEntity->m_pointCloud));
            logInfo += ((CPointCloud*)entity)->m_strCName + " ";
        }
    }

    if (clouds.size() != 2) {
        logInfo += "对齐需要两个点云!";
        m_pMainWin->getPWinVtkPresetWidget()->setWidget(logInfo);
        return;
    }

    // 滤波去噪
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2_filter;
    int cnt = FilterCount(cloud2_filter);
    for(int i = 0;i < cnt;i++){
        cloud2_filter = onFilter(clouds[1]);
        clouds[1] = cloud2_filter;
    }

    if (cloud2_filter->empty()) {
        logInfo += "去噪后点云为空!";
        m_pMainWin->getPWinVtkPresetWidget()->setWidget(logInfo);
        return;
    }

    // 用于采样的两个点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampledCloud1(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampledCloud2(new pcl::PointCloud<pcl::PointXYZRGB>());
    float radius1 = calculateSamplingRadius(clouds[0]);
    float radius2 = calculateSamplingRadius(clouds[1]);

    // 如果点云较小，则不进行采样
    if(radius2 >= 0.1f && radius1 >= 0.1f){
        // 如果点云较大，则动态设置采样半径
        float initialRadius = 0.01f; // 初始采样半径
        float maxRadius = 0.2f;  // 最大采样半径
        float targetSize = 10000;    // 目标点云大小
        float reductionFactor = 1.0 / 5.0; // 点云缩减比例
        // 初始化当前待采样的点云和采样半径
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr currentSource = clouds[1];
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr currentTarget = clouds[0];
        float currentRadius = initialRadius;

        while (true) {
            // 均匀下采样
            pcl::UniformSampling<pcl::PointXYZRGB> uniformSampling;
            uniformSampling.setRadiusSearch(currentRadius);
            uniformSampling.setInputCloud(currentSource);
            uniformSampling.filter(*downsampledCloud2);
            uniformSampling.setInputCloud(currentTarget);
            uniformSampling.filter(*downsampledCloud1);

            if (downsampledCloud1->empty() || downsampledCloud2->empty()) {
                logInfo += "下采样后的点云为空";
                m_pMainWin->getPWinVtkPresetWidget()->setWidget(logInfo);
                return;
            }
            // 检查采样后的点云大小，满足条件则结束采样
            if (downsampledCloud2->size() <= targetSize ||
                downsampledCloud2->size() <= cloud2->size() * reductionFactor) {
                break;
            }
            // 增加采样半径，更新当前源点云和目标点云
            currentRadius += 0.01f;
            currentSource = downsampledCloud2;
            currentTarget = downsampledCloud1;
            if (currentRadius > maxRadius) {
                break;
            }
        }
    }

    if (downsampledCloud1->empty() || downsampledCloud2->empty()) {
        logInfo += "下采样后的点云为空";
        m_pMainWin->getPWinVtkPresetWidget()->setWidget(logInfo);
        return;
    }
    // 保存采样后的点云到文件
    pcl::io::savePCDFileASCII("downsampled_cloud2.pcd", *downsampledCloud2);

    // 使用XYZRGB类型进行ICP配准
    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    icp.setInputSource(downsampledCloud2);
    icp.setInputTarget(downsampledCloud1);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-8);
    icp.setMaxCorrespondenceDistance(0.2f);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr icpFinalCloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>());
    icp.align(*icpFinalCloudPtr);

    // 如果仍未收敛，则进行动态迭代
    if (!icp.hasConverged()) {
        int maxIter = 400; // 最大迭代次数
        int iniIter = 100; // 初始迭代次数
        float corDis = 1.0f; // 最大点距离阈值
        while(iniIter <= maxIter){
            icp.setMaxCorrespondenceDistance(corDis);
            icp.setMaximumIterations(iniIter);
            icp.align(*icpFinalCloudPtr);
            iniIter += 50;
            if(corDis <= 3.0f)  corDis += 0.5f;
        }
    }
    if (!icp.hasConverged()) {
        logInfo += "对齐失败";
        m_pMainWin->getPWinVtkPresetWidget()->setWidget(logInfo);
        return;
    }

    // 处理对齐后的点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr alignedCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    *alignedCloud = *icpFinalCloudPtr;
    auto cloudEntity = m_pMainWin->getPointCloudListMgr()->CreateAlignCloud(alignedCloud);
    m_pMainWin->getPWinToolWidget()->addToList(cloudEntity);
    m_pMainWin->NotifySubscribe();
    //double rmse = icp.getFitnessScore();
    // 添加日志输出
    logInfo += "对齐完成，误差:";
    //logInfo += std::to_string(rmse);
    m_pMainWin->getPWinVtkPresetWidget()->setWidget(logInfo);
}

void sacAlign(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud2){
    // 分别计算两个点云的法向量
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normalsEtimation;
    pcl::PointCloud<pcl::Normal>::Ptr normals1(new pcl::PointCloud<pcl::Normal>());
    pcl::PointCloud<pcl::Normal>::Ptr normals2(new pcl::PointCloud<pcl::Normal>());
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
    normalsEtimation.setInputCloud(cloud1);
    normalsEtimation.setSearchMethod(tree);
    normalsEtimation.setRadiusSearch(50);
    normalsEtimation.compute(*normals1);
    normalsEtimation.setInputCloud(cloud2);
    normalsEtimation.compute(*normals2);

    // 计算FPFH特征
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr featureCloud1(new pcl::PointCloud<pcl::FPFHSignature33>());
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr featureCloud2(new pcl::PointCloud<pcl::FPFHSignature33>());
    pcl::FPFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::FPFHSignature33> fpfh_estimation;
    fpfh_estimation.setInputCloud(cloud1);
    fpfh_estimation.setInputNormals(normals1);
    fpfh_estimation.setSearchMethod(tree);
    fpfh_estimation.setRadiusSearch(50);
    fpfh_estimation.compute(*featureCloud1);
    fpfh_estimation.setInputCloud(cloud2);
    fpfh_estimation.setInputNormals(normals2);
    fpfh_estimation.setSearchMethod(tree);
    fpfh_estimation.setRadiusSearch(50);
    fpfh_estimation.compute(*featureCloud2);

    // SAC-IA粗配准
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZRGB, pcl::PointXYZRGB, pcl::FPFHSignature33> sac_ia;
    sac_ia.setInputSource(cloud1);
    sac_ia.setInputTarget(cloud2);
    sac_ia.setSourceFeatures(featureCloud2);
    sac_ia.setTargetFeatures(featureCloud1);
    sac_ia.setMaximumIterations(100);  // 设置最大迭代次数
    sac_ia.setMinSampleDistance(0.01f);  // 设置最小采样距离
    sac_ia.setMaxCorrespondenceDistance(0.1f);  // 设置最大对应点距离
    pcl::PointCloud<pcl::PointXYZRGB> sacAlignedCloud;
    sac_ia.align(sacAlignedCloud);  // 粗配准
    if (!sac_ia.hasConverged()) {
        return;
    }
}


void VtkWidget::poissonReconstruction()
{
    // 创建曲面实体
    CSurfaces* pSurfaces = (CSurfaces*)m_pMainWin->CreateEntity(enSurfaces);

    auto& entityList = m_pMainWin->m_EntityListMgr->getEntityList();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    QString logInfo;

    // 收集选中的点云（确保不修改原始实体）
    for (int i = 0; i < entityList.size(); i++) {
        CEntity* entity = entityList[i];
        if (!entity->IsSelected()) continue;
        if (entity->GetUniqueType() == enPointCloud) {
            // 获取点云的共享指针，确保原数据不被释放
            auto pcEntity = static_cast<CPointCloud*>(entity);
            pSurfaces->setPointCloud(pcEntity->m_pointCloud);
            cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>(pcEntity->m_pointCloud);
            logInfo += ((CPointCloud*)entity)->m_strAutoName + ' '; // 添加点云编号
        }
    }

    if (cloud->empty()) {
        QMessageBox::warning(this, "警告", "点云不能为空");
        return;
    }

    // ===== 预处理优化 =====
    // 1. 统计去噪（移除离群点）
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(1.0);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    sor.filter(*cloud_filtered);

    // 2. 降采样（保持形状同时减少点数）
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setInputCloud(cloud_filtered);
    vg.setLeafSize(0.003f, 0.003f, 0.003f);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZRGB>);
    vg.filter(*cloud_downsampled);

    // ===== 法向量计算优化 =====
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normalEstimation;
    normalEstimation.setViewPoint(0.0f, 0.0f, 0.0f); // 设置一致视点
    normalEstimation.setInputCloud(cloud_downsampled);

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    normalEstimation.setSearchMethod(tree);

    // 使用K近邻搜索替代半径搜索，更稳定
    normalEstimation.setKSearch(30);  // 调整K值控制局部区域大小
    normalEstimation.compute(*normals);

    // ===== 点云法向量对齐 =====
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloudWithNormals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::concatenateFields(*cloud_downsampled, *normals, *cloudWithNormals);

    // 法向量方向一致性调整
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_oriented(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::copyPointCloud(*cloudWithNormals, *cloud_oriented);

    for (auto& point : *cloud_oriented) {
        // 强制法向量指向视点方向（假设视点位于点云中心）
        Eigen::Vector3f view_point(0, 0, 0); // 根据实际视点调整
        Eigen::Vector3f point_pos(point.x, point.y, point.z);
        Eigen::Vector3f normal_dir(point.normal_x, point.normal_y, point.normal_z);

        if ((point_pos - view_point).dot(normal_dir) < 0) {
            normal_dir *= -1;
            point.normal_x = normal_dir[0];
            point.normal_y = normal_dir[1];
            point.normal_z = normal_dir[2];
        }
    }

    // ===== 泊松重建参数优化 =====
    pcl::Poisson<pcl::PointXYZRGBNormal> poisson;
    poisson.setDepth(8);          // 增加深度防止过拟合
    poisson.setSolverDivide(8);   // 保持求解精度
    poisson.setIsoDivide(8);      // 保持等值面划分密度
    poisson.setSamplesPerNode(5); // 保持采样密度
    poisson.setScale(0.9f);        // 移除人工缩放
    //    poisson.setLinearFit(true);   // 启用线性拟合增强局部一致性
    poisson.setInputCloud(cloud_oriented);

    pcl::PolygonMesh mesh;
    poisson.reconstruct(mesh);

    // ===== 后处理优化 =====
    // 网格平滑（可选）
    pcl::MeshSmoothingLaplacianVTK meshSmoothing;
    pcl::PolygonMeshConstPtr mesh_ptr = pcl::make_shared<const pcl::PolygonMesh>(mesh);
    meshSmoothing.setInputMesh(mesh_ptr);
    meshSmoothing.setNumIter(5);  // 减少迭代次数保持细节
    pcl::PolygonMesh smoothedMesh;
    meshSmoothing.process(smoothedMesh);

    if (smoothedMesh.polygons.empty()) {
        QMessageBox::warning(this, "警告", "泊松重建失败");
        return;
    }

    // 重建曲面属性设置
    vtkSmartPointer<vtkPolyData> vtkMesh = convertPclMeshToVtkPolyData(smoothedMesh);
    pSurfaces->setCurrentId();
    pSurfaces->Form = "重建";
    pSurfaces->setMesh(vtkMesh);
    m_pMainWin->getPWinToolWidget()->addToList(pSurfaces);
    pSurfaces->m_CreateForm = eReconstruct;

    m_pMainWin->NotifySubscribe();

    // 添加日志输出
    logInfo += "泊松重建完成";
    m_pMainWin->getPWinVtkPresetWidget()->setWidget(logInfo);
}

float VtkWidget::calculateSamplingRadius(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{
    // 根据点云的大小动态设置采样半径
    if (cloud->size() < 100000) { // 小于10000个点的小点云
        return 0.01f;
    } else if (cloud->size() < 500000 && cloud->size() > 100000) {
        return 0.05f;
    } else if (cloud->size() < 1000000 && cloud->size() >= 500000) {
        return 0.1f;
    } else {
        return 0.2f;
    }
}

vtkSmartPointer<vtkPolyData> VtkWidget::convertPclMeshToVtkPolyData(const pcl::PolygonMesh& pclMesh) {
    vtkSmartPointer<vtkPolyData> vtkMesh = vtkSmartPointer<vtkPolyData>::New();

    // 提取顶点坐标和颜色
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromPCLPointCloud2(pclMesh.cloud, cloud);

    vtkNew<vtkPoints> points;
    vtkNew<vtkUnsignedCharArray> colors;
    colors->SetNumberOfComponents(3);
    colors->SetName("Colors");

    for (const auto& point : cloud) {
        points->InsertNextPoint(point.x, point.y, point.z);
        // unsigned char color[3] = {point.r, point.g, point.b};
        unsigned char color[3] = {200, 200,200}; // 设置颜色
        colors->InsertNextTypedTuple(color);
    }

    // 提取面片信息
    vtkNew<vtkCellArray> polygons;
    for (const auto& polygon : pclMesh.polygons) {
        vtkNew<vtkIdList> idList;
        for (const auto& vertex : polygon.vertices) {
            idList->InsertNextId(static_cast<vtkIdType>(vertex));
        }
        polygons->InsertNextCell(idList);
    }

    // 组装vtkPolyData
    vtkMesh->SetPoints(points);
    vtkMesh->SetPolys(polygons);
    vtkMesh->GetPointData()->SetScalars(colors); // 附加颜色

    return vtkMesh;
}
