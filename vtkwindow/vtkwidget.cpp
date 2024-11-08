#include "vtkwindow/vtkwidget.h"
#include <vtkInteractorStyle.h>
#include <vtkEventQtSlotConnect.h>
#include <QFileDialog>  // 用于文件对话框
#include <QOpenGLContext>
#include <qopenglfunctions.h>
#include <QMessageBox>


// 渲染窗口大小
#define WIDTH 1000
#define HEIGHT 800

VtkWidget::VtkWidget(QWidget *parent)
    : QWidget(parent),
    cloud1(new pcl::PointCloud<pcl::PointXYZ>()),  // 初始化第一个点云对象
    cloud2(new pcl::PointCloud<pcl::PointXYZ>()), // 初始化第二个点云对象
    comparisonCloud(new pcl::PointCloud<pcl::PointXYZRGB>()) // 初始化比较好的点云对象
{
    // 禁用 VTK 的错误处理弹窗
    vtkObject::GlobalWarningDisplayOff();
    m_pMainWin = (MainWindow*) parent;

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
    auto m_highlightstyle = vtkSmartPointer<MouseInteractorHighlightActor>::New();
    m_highlightstyle->SetRenderer(renderer);
    m_highlightstyle->SetUpMainWin(m_pMainWin);
    renWin->GetInteractor()->SetInteractorStyle(m_highlightstyle);

    // auto customstyle = vtkSmartPointer<CustomInteractorStyle>::New();
    // customstyle->SetRenderer(renderer);
    // renWin->GetInteractor()->SetInteractorStyle(customstyle);

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

vtkSmartPointer<vtkRenderWindow> VtkWidget::getRenderWindow(){
    return renWin;
}

vtkSmartPointer<vtkRenderer>& VtkWidget::getRenderer(){
    // return m_renderer;
    return renderer;
}

// 刷新vtk窗口
void VtkWidget::UpdateInfo(){
    reDrawCentity();
    reDrawCloud();
}

void VtkWidget::reDrawCentity(){
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

    // 遍历entitylist绘制图形并加入渲染器
    for(auto i = 0;i < entitylist.size();i++){
        int flag=0;
        if(constructEntityList.isEmpty()){//没有构建的元素

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

void VtkWidget::reDrawCloud()
{
    showConvertedCloud();
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


    auto cloud_rgb_1(new pcl::PointCloud<pcl::PointXYZRGB>);
    // 分别用迭代器遍历两个map的所有文件
    for(auto item = measured_map.begin(); item != measured_map.end(); item++){
        // 如果文件不隐藏
        if(item.value()){
            pcl::io::loadPCDFile(item.key().toStdString(), *cloud_rgb_1);

            // 将临时点云指针传给toolwidget，暂时查看拟合功能
            m_pMainWin->getpWinFileMgr()->cloudptr=pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>(cloud_rgb_1);

            // 将cloud转换为VTK的点集
            vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
            vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
            colors->SetNumberOfComponents(3);
            colors->SetName("Colors");

            points->SetNumberOfPoints(cloud_rgb_1->points.size());
            for (size_t i = 0; i < cloud_rgb_1->points.size(); ++i)
            {
                points->SetPoint(i, cloud_rgb_1->points[i].x, cloud_rgb_1->points[i].y, cloud_rgb_1->points[i].z);
            }

            vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
            polyData->SetPoints(points);
            polyData->GetPointData()->SetScalars(colors);

            // 创建一个顶点过滤器来生成顶点表示
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

            // getCloudActors().push_back(actor); // 将转化后的点云存入列表
            renderer->AddActor(actor);
        }
    }

    pcl::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud_2 = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    //auto cloud_2(new pcl::PointCloud<pcl::PointXYZRGB>);
    for(auto item = model_map.begin();item != model_map.end() ;item++){
        // 如果文件不隐藏
        if(item.value()){
            pcl::io::loadPLYFile(item.key().toStdString(), *cloud_2);
            // 将加载的点云存入列表
            m_pMainWin->getpWinFileMgr()->cloudptr=cloud_2;

            // 将cloud转换为VTK的点集
            vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
            vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
            colors->SetNumberOfComponents(3);
            colors->SetName("Colors");

            points->SetNumberOfPoints(cloud_2->points.size());
            for (size_t i = 0; i < cloud_2->points.size(); ++i)
            {
                points->SetPoint(i, cloud_2->points[i].x, cloud_2->points[i].y, cloud_2->points[i].z);
            }

            vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
            polyData->SetPoints(points);
            polyData->GetPointData()->SetScalars(colors);

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

    getRenderWindow()->Render(); // 刷新渲染窗口
}

void VtkWidget::showConvertedCloud(pcl::PointCloud<pcl::PointXYZRGB> cloud_rgb_1){
    // 将cloud转换为VTK的点集
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
    colors->SetNumberOfComponents(3);
    colors->SetName("Colors");

    points->SetNumberOfPoints(cloud_rgb_1.points.size());
    for (size_t i = 0; i < cloud_rgb_1.points.size(); ++i)
    {
        points->SetPoint(i, cloud_rgb_1.points[i].x, cloud_rgb_1.points[i].y, cloud_rgb_1.points[i].z);
        unsigned char r = static_cast<unsigned char>(cloud_rgb_1.points[i].r);
        unsigned char g = static_cast<unsigned char>(cloud_rgb_1.points[i].g);
        unsigned char b = static_cast<unsigned char>(cloud_rgb_1.points[i].b);
        colors->InsertNextTuple3(r, g, b);
    }

    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);
    polyData->GetPointData()->SetScalars(colors);

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
    actor->GetProperty()->SetPointSize(6); // 设置点大小

    renderer->AddActor(actor);
    getRenderWindow()->Render(); // 刷新渲染窗口
}

// 比较两个点云的处理函数
void VtkWidget::onCompare()
{
    // 获取打开的模型文件和实测文件
    auto file_model = m_pMainWin->getpWinFileMgr()->getModelFileMap().lastKey();
    auto file_measure = m_pMainWin->getpWinFileMgr()->getMeasuredFileMap().lastKey();

    // 初始化两个点云
    pcl::io::loadPLYFile(file_model.toStdString(), *cloud1);
    pcl::io::loadPCDFile(file_measure.toStdString(), *cloud2);

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

// //FPFH(粗配准)+ICP(精配准)
// void VtkWidget::onAlign()
// {
//     // 获取打开的模型文件和实测文件
//     auto file_model = m_pMainWin->getpWinFileMgr()->getModelFileMap().firstKey();
//     auto file_measure = m_pMainWin->getpWinFileMgr()->getModelFileMap().lastKey();

//     // 初始化两个点云
//     pcl::io::loadPLYFile(file_model.toStdString(), *cloud1);
//     pcl::io::loadPLYFile(file_measure.toStdString(), *cloud2);

//     // 检查点云是否为空
//     if (cloud1->empty() || cloud2->empty()) {
//         QMessageBox::warning(this, "Warning", "One or both point clouds are empty!");
//         return;
//     }

//     // 下采样：提高计算效率，对输入点云进行下采样
//     pcl::PointCloud<pcl::PointXYZ>::Ptr downsampledCloud1(new pcl::PointCloud<pcl::PointXYZ>());
//     pcl::PointCloud<pcl::PointXYZ>::Ptr downsampledCloud2(new pcl::PointCloud<pcl::PointXYZ>());
//     pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
//     voxelGrid.setLeafSize(0.05f, 0.05f, 0.05f);  // 设置叶子大小为 5cm
//     voxelGrid.setInputCloud(cloud1);
//     voxelGrid.filter(*downsampledCloud1);
//     voxelGrid.setInputCloud(cloud2);
//     voxelGrid.filter(*downsampledCloud2);

//     // 计算法线
//     pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
//     pcl::PointCloud<pcl::Normal>::Ptr normals1(new pcl::PointCloud<pcl::Normal>());
//     pcl::PointCloud<pcl::Normal>::Ptr normals2(new pcl::PointCloud<pcl::Normal>());
//     pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
//     normalEstimation.setSearchMethod(tree);
//     normalEstimation.setRadiusSearch(0.05);  // 设置法线估计的半径

//     normalEstimation.setInputCloud(downsampledCloud1);
//     normalEstimation.compute(*normals1);
//     normalEstimation.setInputCloud(downsampledCloud2);
//     normalEstimation.compute(*normals2);

//     // 计算 FPFH 特征
//     pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfhEstimation;
//     pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs1(new pcl::PointCloud<pcl::FPFHSignature33>());
//     pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs2(new pcl::PointCloud<pcl::FPFHSignature33>());

//     fpfhEstimation.setSearchMethod(tree);
//     fpfhEstimation.setRadiusSearch(0.1);  // 设置特征估计的半径

//     fpfhEstimation.setInputCloud(downsampledCloud1);
//     fpfhEstimation.setInputNormals(normals1);
//     fpfhEstimation.compute(*fpfhs1);

//     fpfhEstimation.setInputCloud(downsampledCloud2);
//     fpfhEstimation.setInputNormals(normals2);
//     fpfhEstimation.compute(*fpfhs2);

//     // 使用 RANSAC 进行初步配准
//     pcl::SampleConsensusPrerejective<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac;
//     sac.setInputSource(downsampledCloud1);
//     sac.setSourceFeatures(fpfhs1);
//     sac.setInputTarget(downsampledCloud2);
//     sac.setTargetFeatures(fpfhs2);
//     sac.setMaximumIterations(1000);  // 设置最大迭代次数
//     sac.setNumberOfSamples(3);  // 使用 3 个点作为采样
//     sac.setCorrespondenceRandomness(5);  // 设置随机对应点数量
//     sac.setSimilarityThreshold(0.9f);  // 设置相似度阈值
//     sac.setMaxCorrespondenceDistance(0.1);  // 设置最大对应点距离
//     sac.setInlierFraction(0.25f);  // 最少内点比例

//     pcl::PointCloud<pcl::PointXYZ> sacAlignedCloud;
//     sac.align(sacAlignedCloud);

//     // 检查 RANSAC 是否成功收敛
//     if (!sac.hasConverged()) {
//         QMessageBox::critical(this, "Error", "FPFH-based coarse alignment did not converge!");
//         return;
//     }

//     // 获取初始变换矩阵
//     Eigen::Matrix4f initialTransformation = sac.getFinalTransformation();

//     // 使用 ICP 进行精细对齐
//     pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
//     icp.setInputSource(downsampledCloud1);
//     icp.setInputTarget(downsampledCloud2);
//     icp.setMaximumIterations(50);  // 设置最大迭代次数
//     icp.setTransformationEpsilon(1e-8);  // 设置变换容差
//     icp.setMaxCorrespondenceDistance(0.05);  // 设置最大对应点距离

//     pcl::PointCloud<pcl::PointXYZ> icpFinalCloud;
//     icp.align(icpFinalCloud, initialTransformation);  // 使用初始变换进行 ICP 对齐

//     // 检查 ICP 是否成功收敛
//     if (icp.hasConverged()) {
//         // 计算最终的配准结果并应用于原始点云
//         pcl::PointCloud<pcl::PointXYZ>::Ptr alignedCloud(new pcl::PointCloud<pcl::PointXYZ>());
//         pcl::transformPointCloud(*cloud1, *alignedCloud, icp.getFinalTransformation());

//         // 显示对齐后的点云
//         // showConvertedCloud(alignedCloud, "Aligned Cloud");
//         // 将cloud转换为VTK的点集
//         vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
//         points->SetNumberOfPoints(alignedCloud->points.size());
//         for (size_t i = 0; i < alignedCloud->points.size(); ++i)
//         {
//             points->SetPoint(i, alignedCloud->points[i].x, alignedCloud->points[i].y, alignedCloud->points[i].z);
//         }

//         vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
//         polyData->SetPoints(points);

//         // 创建一个顶点过滤器来生成顶点表示（可选，但通常用于点云）
//         vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
//         glyphFilter->SetInputData(polyData);
//         glyphFilter->Update();

//         polyData = glyphFilter->GetOutput();

//         // 创建映射器并将glyphFilter的几何数据输入
//         vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
//         mapper->SetInputData(polyData);

//         vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
//         actor->SetMapper(mapper);
//         actor->GetProperty()->SetPointSize(6); // 设置点大小
//         actor->GetProperty()->SetColor(0.5, 0.5, 0.5);

//         renderer->AddActor(actor);

//         // 输出 RMSE
//         double rmse = icp.getFitnessScore();
//         QMessageBox::information(this, "Alignment Result", QString("Fine alignment RMSE: %1").arg(rmse));
//     } else {
//         QMessageBox::critical(this, "Error", "ICP fine alignment did not converge!");
//     }
// }

void VtkWidget::onAlign()
{
    // 获取打开的模型文件和实测文件
    auto file_model = m_pMainWin->getpWinFileMgr()->getModelFileMap().lastKey();
    auto file_measure = m_pMainWin->getpWinFileMgr()->getModelFileMap().lastKey();

    // 初始化两个点云
    pcl::io::loadPLYFile(file_model.toStdString(), *cloud1);
    pcl::io::loadPLYFile(file_measure.toStdString(), *cloud2);

    // 检查点云是否为空
    if (cloud1->empty() || cloud2->empty()) {
        QMessageBox::warning(this, "Warning", "One or both point clouds are empty!");
        return;
    }

    // 下采样：提高计算效率，对输入点云进行下采样
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampledCloud1(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampledCloud2(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;
    voxelGrid.setLeafSize(0.05f, 0.05f, 0.05f);  // 设置叶子大小为 5cm
    voxelGrid.setInputCloud(cloud1);
    voxelGrid.filter(*downsampledCloud1);
    voxelGrid.setInputCloud(cloud2);
    voxelGrid.filter(*downsampledCloud2);

    // 使用 ICP 进行对齐
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(downsampledCloud1);
    icp.setInputTarget(downsampledCloud2);
    icp.setMaximumIterations(50);  // 设置最大迭代次数
    icp.setTransformationEpsilon(1e-8);  // 设置变换容差
    icp.setMaxCorrespondenceDistance(0.05);  // 设置最大对应点距离

    pcl::PointCloud<pcl::PointXYZ> icpFinalCloud;
    icp.align(icpFinalCloud);  // 进行 ICP 对齐

    // 检查 ICP 是否成功收敛
    if (icp.hasConverged()) {
        // 计算最终的配准结果并应用于原始点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr alignedCloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::transformPointCloud(*cloud1, *alignedCloud, icp.getFinalTransformation());

        // 显示对齐后的点云
        // 将cloud转换为VTK的点集
        vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
        points->SetNumberOfPoints(alignedCloud->points.size());
        for (size_t i = 0; i < alignedCloud->points.size(); ++i)
        {
            points->SetPoint(i, alignedCloud->points[i].x, alignedCloud->points[i].y, alignedCloud->points[i].z);
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
        actor->GetProperty()->SetPointSize(6); // 设置点大小
        actor->GetProperty()->SetColor(0.5, 0.5, 0.5);

        renderer->AddActor(actor);

        // 输出 RMSE
        double rmse = icp.getFitnessScore();
        QMessageBox::information(this, "Alignment Result", QString("Fine alignment RMSE: %1").arg(rmse));
    } else {
        QMessageBox::critical(this, "Error", "ICP alignment did not converge!");
    }
}
