#include "vtkwindow/vtkwidget.h"
#include <vtkInteractorStyle.h>
#include <QFileDialog>  // 用于文件对话框
#include <QOpenGLContext>
#include <qopenglfunctions.h>
#include <QMessageBox>
#include <pcl/common/common.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <cmath>
#include <limits>
#include <pcl/common/distances.h>  // PCL距离计算函数
#include <pcl/visualization/pcl_visualizer.h>  // PCL可视化库
#include <pcl/registration/icp.h>
#include <pcl/kdtree/kdtree_flann.h>

// 渲染窗口大小
#define WIDTH 1000
#define HEIGHT 800

VtkWidget::VtkWidget(QWidget *parent)
    : QWidget(parent),
    cloud1(new pcl::PointCloud<pcl::PointXYZ>()),  // 初始化第一个点云对象
    cloud2(new pcl::PointCloud<pcl::PointXYZ>()), // 初始化第二个点云对象
    comparisonCloud(new pcl::PointCloud<pcl::PointXYZRGB>()) // 初始化比较好的点云对象
// ,visualizer(new pcl::visualization::PCLVisualizer("Cloud Comparator"))  // 初始化可视化对象
{
    // 禁用 VTK 的错误处理弹窗
    vtkObject::GlobalWarningDisplayOff();
    m_pMainWin = (MainWindow*) parent;
    m_clickstyle = (MouseInteractorHighlightActor*) parent;

    // 设置 VTK 渲染窗口到 QWidget
    QVBoxLayout *mainlayout = new QVBoxLayout(this);
    setUpVtk(mainlayout); // 配置vtk窗口
    this->setLayout(mainlayout);
}

void VtkWidget::setUpVtk(QVBoxLayout *layout){
    // 初始化渲染器和交互器
    renderer = vtkSmartPointer<vtkRenderer>::New();
    renderer->SetBackground(1, 1, 1); // 设置渲染器颜色为白
    renWin = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    // renWin = vtkSmartPointer<vtkRenderWindow>::New();
    renWin->AddRenderer(renderer);  // 将渲染器添加到渲染窗口

    // 创建QVTKOpenGLNativeWidget作为渲染窗口
    vtkWidget = new QVTKOpenGLNativeWidget();
    layout->addWidget(vtkWidget);
    vtkWidget->setRenderWindow(renWin);

    // 添加交互器样式
    auto clickstyle = vtkSmartPointer<MouseInteractorHighlightActor>::New();
    clickstyle->SetRenderer(renderer);
    renWin->GetInteractor()->SetInteractorStyle(clickstyle);


    // 创建坐标器
    createAxes();

    // 创建初始视角相机
    vtkCamera* camera = renderer->GetActiveCamera();
    if (camera) {
        // 设置相机的初始位置和焦点
        camera->SetPosition(0, 0, 1);
        camera->SetFocalPoint(0, 0, 0);
        camera->SetViewUp(0, 1, 0);

        // 根据需要调整视野角度
        // camera->SetViewAngle(60);

        // 根据场景的具体大小和需要调整裁剪范围
        // camera->SetClippingRange(0.1, 1000);

        camera->OrthogonalizeViewUp(); // 确保与SetViewUp方向正交
        // 根据需要调整视图的缩放
        camera->Zoom(0.5);
    }

    getRenderWindow()->Render();

}

// 配置点云的相关
void VtkWidget::setUpPcl()
{

}

vtkSmartPointer<vtkRenderWindow> VtkWidget::getRenderWindow(){
    return renWin;
}

vtkSmartPointer<vtkRenderer>& VtkWidget::getRenderer(){
    // return m_renderer;
    return renderer;
}

void VtkWidget::UpdateInfo(){
    reDraw();
    showConvertedCloud();
}

void VtkWidget::reDraw(){
    // 获取渲染器中的所有 actor
    auto* actorCollection = getRenderer()->GetViewProps();

    // 创建一个迭代器用于遍历actor集合
    vtkCollectionSimpleIterator it;

    // 初始化迭代器，准备遍历actor集合
    actorCollection->InitTraversal(it);

    vtkSmartPointer<vtkProp> prop;
    // 遍历并移除 vtkProp 对象（包括 vtkActor 和 vtkAxesActor）
    while ((prop = actorCollection->GetNextProp(it)) != nullptr)
    {
        renderer->RemoveViewProp(prop);
    }

    auto entitylist = m_pMainWin->m_EntityListMgr->getEntityList();
    auto objectlist = m_pMainWin->m_ObjectListMgr->getObjectList();
    QVector<bool> list = m_pMainWin->m_EntityListMgr->getMarkList();//获取标记是否隐藏元素的list
    QMap<QString, bool> filemap = m_pMainWin->getpWinFileMgr()->getContentItemMap();
    QVector<CEntity*> constructEntityList = m_pMainWin->getPWinToolWidget()->getConstructEntityList();//存储构建元素的列表
    auto pickedActors = m_clickstyle->getPickedActors(); // 获取选中高亮的actor


    // 遍历entitylist绘制图形并加入渲染器
    for(auto i = 0;i < entitylist.size();i++){
        int flag=0;
        if(constructEntityList.isEmpty()){//没有构建的元素
            // vtkActor* entity_actor = entitylist[i]->draw();
            // // 遍历pickedActors，如果entitylist中有选中的成员则保持选中状态
            // for(auto &pair : pickedActors){
            //     // 判断entity_actor属性
            //     if(entity_actor->GetProperty() == pair.second){
            //         m_clickstyle->HighlightActor(entity_actor); // 高亮显示
            //         getRenderer()->AddActor(entity_actor);
            //     }
            // }
            getRenderer()->AddActor(entitylist[i]->draw());
        }
        else{
            for(int j=0;j<constructEntityList.size();j++){
                QString key=constructEntityList[j]->GetObjectCName() + "  " + constructEntityList[j]->GetObjectAutoName();
                if(entitylist[i] == constructEntityList[j]){//是构建的元素
                    flag=1;
                    if(filemap[key]){
                        getRenderer()->AddActor(entitylist[i]->draw());
                        break;
                    }
                }
            }
            if(flag==0){//不是构建的元素
                getRenderer()->AddActor(entitylist[i]->draw());
            }
        }
    }

    // 遍历objectlist绘制坐标系并加入渲染器
    for(auto object:objectlist){
        if(object)
            getRenderer()->AddActor(object->draw());
    }

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

    orientationWidget = vtkSmartPointer<vtkOrientationMarkerWidget>::New();
    // 将坐标轴演员添加到orientationWidget
    orientationWidget->SetOrientationMarker(axesActor);
    // 将orientationWidget与交互器关联
    orientationWidget->SetInteractor(renWin->GetInteractor());
    // 设置视口
    orientationWidget->SetViewport(0.0, 0.0, 0.2, 0.2);

    orientationWidget->SetEnabled(1);
    orientationWidget->InteractiveOn();
}

// 切换相机视角1
void VtkWidget::onTopView() {
    vtkCamera *camera = renderer->GetActiveCamera();
    if (camera) {
        camera->SetPosition(0, 0, 1);  // 重置相机位置为俯视
        camera->SetFocalPoint(0, 0, 0);
        camera->SetViewUp(0, 1, 0);

        camera->OrthogonalizeViewUp(); // 确保与SetViewUp方向正交

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

        camera->Azimuth(30);
        camera->Elevation(30);
        camera->OrthogonalizeViewUp();

        // 重新设置相机并渲染
        renderer->ResetCamera();
        renWin->Render();
    }
}

// 显示要测量的点云图像和模型点云
void VtkWidget::showConvertedCloud(){
    // 获取待测量的点云文件map
    auto measured_map = m_pMainWin->getpWinFileMgr()->getMeasuredFileMap();
    auto model_map = m_pMainWin->getpWinFileMgr()->getModelFileMap();

    // 分别用迭代器遍历两个map的所有文件
    auto cloud_1(new pcl::PointCloud<pcl::PointXYZ>);
    for(auto item = measured_map.begin();item != measured_map.end() ;item++){
        // 如果文件不隐藏
        if(item.value()){
            pcl::io::loadPCDFile(item.key().toStdString(), *cloud_1);

            // 将cloud转换为VTK的点集
            vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
            points->SetNumberOfPoints(cloud_1->points.size());
            for (size_t i = 0; i < cloud_1->points.size(); ++i)
            {
                points->SetPoint(i, cloud_1->points[i].x, cloud_1->points[i].y, cloud_1->points[i].z);
            }

            vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
            polyData->SetPoints(points);

            // 创建一个顶点过滤器来生成顶点表示（可选，但通常用于点云）
            vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
            glyphFilter->SetInputData(polyData);
            glyphFilter->Update();

            polyData = glyphFilter->GetOutput();

            // 创建映射器并将glyphFilter的几何数据输入
            vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
            mapper->SetInputData(polyData);

            vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
            actor->SetMapper(mapper);
            actor->GetProperty()->SetPointSize(5); // 设置点大小
            actor->GetProperty()->SetColor(0.5, 0.5, 0.5);

            renderer->AddActor(actor);
        }
    }

    auto cloud_2(new pcl::PointCloud<pcl::PointXYZ>);
    for(auto item = model_map.begin();item != model_map.end() ;item++){
        // 如果文件不隐藏
        if(item.value()){
            pcl::io::loadPLYFile(item.key().toStdString(), *cloud_1);

            // 将cloud转换为VTK的点集
            vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
            points->SetNumberOfPoints(cloud_2->points.size());
            for (size_t i = 0; i < cloud_1->points.size(); ++i)
            {
                points->SetPoint(i, cloud_1->points[i].x, cloud_1->points[i].y, cloud_1->points[i].z);
            }

            vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
            polyData->SetPoints(points);

            // 创建一个顶点过滤器来生成顶点表示（可选，但通常用于点云）
            vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
            glyphFilter->SetInputData(polyData);
            glyphFilter->Update();

            polyData = glyphFilter->GetOutput();

            // 创建映射器并将glyphFilter的几何数据输入
            vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
            mapper->SetInputData(polyData);

            vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
            actor->SetMapper(mapper);
            actor->GetProperty()->SetPointSize(5); // 设置点大小
            actor->GetProperty()->SetColor(0.3, 0.3, 0.3);

            renderer->AddActor(actor);
        }
    }

    getRenderWindow()->Render(); // 刷新渲染窗口
}

// void VtkWidget::showConvertedCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const std::string &name){
// }

// 比较两个点云的处理函数
void VtkWidget::onCompare()
{
    // 获取打开的模型文件和实测文件
    // auto file_model = m_pMainWin->getpWinFileMgr()->getModelFileMap().firstKey();
    auto file_model = m_pMainWin->getpWinFileMgr()->getMeasuredFileMap().firstKey();
    auto file_measure = m_pMainWin->getpWinFileMgr()->getMeasuredFileMap().lastKey();

    // 初始化两个点云
    pcl::io::loadPCDFile(file_model.toStdString(), *cloud1);
    pcl::io::loadPCDFile(file_measure.toStdString(), *cloud2);

    // 检查点云是否为空
    if (cloud1->empty() || cloud2->empty()) {
        QMessageBox::warning(this, "Warning", "One or both point clouds are empty!");
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


    // 转为vtk带颜色的点集，并直接显示
    // 创建一个新的VTK点集对象，并设置点的数量
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    points->SetNumberOfPoints(comparisonCloud->points.size());

    // 创建一个新的VTK无符号字符数组对象，用于存储颜色信息
    // 设置颜色数组的组件数（RGB）和元组数（点的数量）
    vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
    colors->SetNumberOfComponents(3);
    colors->SetNumberOfTuples(comparisonCloud->points.size());

    // 创建一个新的VTK单元格数组对象，用于存储顶点信息
    vtkSmartPointer<vtkCellArray> vertexCells = vtkSmartPointer<vtkCellArray>::New();

    // 遍历PCL点云中的每个点
    for (size_t i = 0; i < comparisonCloud->points.size(); ++i)
    {
        // 设置VTK点集中的点的位置
        points->SetPoint(i, comparisonCloud->points[i].x, comparisonCloud->points[i].y, comparisonCloud->points[i].z);
        // 设置颜色数组中的颜色值（RGB）
        colors->SetTuple3(i, comparisonCloud->points[i].r, comparisonCloud->points[i].g, comparisonCloud->points[i].b);
    }

    // 创建一个新的VTK多边形数据对象
    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);

    // 创建一个顶点过滤器来生成顶点表示（可选，但通常用于点云）
    vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
    glyphFilter->SetInputData(polyData);
    glyphFilter->Update();

    polyData = glyphFilter->GetOutput();

    // 将点集、颜色数组和顶点单元格数组设置到多边形数据对象中
    polyData->SetPoints(points);
    polyData->GetPointData()->SetScalars(colors); // 设置颜色为点数据中的标量

    // Create a mapper and actor
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(polyData);

    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetPointSize(5);

    renderer->AddActor(actor);
    renWin->Render();
}

void VtkWidget::onAlign()
{
    // 检查点云是否为空
    if (cloud1->empty() || cloud2->empty()) {
        QMessageBox::warning(this, "Warning", "One or both point clouds are empty!");
        return;
    }

    // 创建一个新的点云对象用于存储对齐后的点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr alignedCloud(new pcl::PointCloud<pcl::PointXYZ>());

    // 创建 ICP 对象并设置参数
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud1);
    icp.setInputTarget(cloud2);
    icp.setMaximumIterations(50);  // 设置最大迭代次数
    icp.setTransformationEpsilon(1e-8);  // 设置变换的容差

    // 执行 ICP 配准
    pcl::PointCloud<pcl::PointXYZ> finalCloud;
    icp.align(finalCloud);

    // 检查配准是否成功
    if (icp.hasConverged()) {
        *alignedCloud = finalCloud;
        // showConvertedCloud(alignedCloud, "Aligned Cloud");  // 显示对齐后的点云
    } else {
        QMessageBox::critical(this, "Error", "ICP did not converge!");
        return;
    }

    // 可选：计算和对齐结果进行评估，如计算 RMSE（均方根误差）
    double rmse = icp.getFitnessScore();
    QMessageBox::information(this, "Alignment Result", QString("RMSE: %1").arg(rmse));

}
