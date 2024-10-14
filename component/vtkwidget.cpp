#include "vtkwidget.h"
#include <vtkInteractorStyle.h>

#include <QFileDialog>  // 用于文件对话框
#include <QMessageBox>  // 用于消息框
#include <pcl/common/common.h>  // PCL基础库
#include <pcl/io/ply_io.h>  // PCL PLY文件读写
#include <pcl/io/pcd_io.h>  // PCL PCD文件读写
#include <cmath>  // 数学函数库
#include <limits>  // 极限值定义
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
    cloud2(new pcl::PointCloud<pcl::PointXYZ>())  // 初始化第二个点云对象
    // ,visualizer(new pcl::visualization::PCLVisualizer("Cloud Comparator"))  // 初始化可视化对象
{
    m_pMainWin = (MainWindow*) parent;

    // 设置 VTK 渲染窗口到 QWidget
    QVBoxLayout *mainlayout = new QVBoxLayout;
    setUpVtk(mainlayout); // 配置vtk窗口
    // setUpPcl(); // 配置点云
    this->setLayout(mainlayout);

}

void VtkWidget::setUpVtk(QVBoxLayout *layout){
    // 初始化渲染器和交互器
    renWin = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    renderer = vtkSmartPointer<vtkRenderer>::New();
    renderer->SetBackground(1, 1, 1); // 设置渲染器颜色为白
    renWin->AddRenderer(renderer);  // 将渲染器添加到渲染窗口

    // 从visualizer得到渲染窗口
    // renWin = visualizer->getRenderWindow();
    // renderer = vtkSmartPointer<vtkRenderer>::New();
    // renWin->AddRenderer(renderer);  // 将渲染器添加到渲染窗口

    // 添加交互器
    interactor = vtkSmartPointer<vtkGenericRenderWindowInteractor>::New();
    // interactor->SetRenderWindow(renWin);

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

    // // 创建QVTKOpenGLNativeWidget作为渲染窗口
    // vtkWidget = new QVTKOpenGLNativeWidget(this);
    // vtkWidget->setRenderWindow(renWin);
    // visualizer->setupInteractor(interactor, renWin);
    // // 设置布局并将qvtkWidget添加到其中
    // layout->addWidget(vtkWidget);

    vtkWidget = new QVTKOpenGLNativeWidget(this);
    vtkWidget->setRenderWindow(renWin);
    layout->addWidget(vtkWidget);

    getRenderWindow()->Render();
}

// 配置点云的相关
void VtkWidget::setUpPcl()
{
    // // 加载点云文件1
    // QString fileName = QFileDialog::getOpenFileName(this, "Open Point Cloud File 1", "", "PCD Files (*.pcd);;PLY Files (*.ply)");
    // if (fileName.isEmpty()) return;  // 如果文件名为空，直接返回

    // // 根据文件后缀加载不同的点云文件
    // if (fileName.endsWith(".pcd")) {
    //     if (pcl::io::loadPCDFile<pcl::PointXYZ>(fileName.toStdString(), *cloud1) == -1) {
    //         QMessageBox::critical(this, "Error", "Couldn't read the PCD file!");
    //         return;
    //     }
    // } else if (fileName.endsWith(".ply")) {
    //     if (pcl::io::loadPLYFile<pcl::PointXYZ>(fileName.toStdString(), *cloud1) == -1) {
    //         QMessageBox::critical(this, "Error", "Couldn't read the PLY file!");
    //         return;
    //     }
    // } else {
    //     QMessageBox::critical(this, "Error", "Unsupported file format!");
    //     return;
    // }

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

    QVector<bool> list = m_pMainWin->m_EntityListMgr->getMarkList();//获取标记是否隐藏元素的list
    // 存储返回的引用对象，用于操作两个list
    auto entitylist = m_pMainWin->m_EntityListMgr->getEntityList();
    auto objectlist = m_pMainWin->m_ObjectListMgr->getObjectList();

    // 遍历entitylist绘制图形并加入渲染器
    for(auto i = 0;i < entitylist.size();i++){
        if(!list[i]){
            getRenderer()->AddActor(entitylist[i]->draw());
        }
    }

    // 遍历objectlist绘制坐标系并加入渲染器
    for(auto object:objectlist){
        if(object)
            getRenderer()->AddActor(object->draw());
    }

    // 重新加载全局坐标系
    axesActor = vtkSmartPointer<vtkAxesActor>::New();

    // 设置 X Y Z 轴标题颜色为黑色
    axesActor->GetXAxisCaptionActor2D()->GetCaptionTextProperty()->SetColor(0.0, 0.0, 0.0);
    axesActor->GetYAxisCaptionActor2D()->GetCaptionTextProperty()->SetColor(0.0, 0.0, 0.0);
    axesActor->GetZAxisCaptionActor2D()->GetCaptionTextProperty()->SetColor(0.0, 0.0, 0.0);

    axesActor->SetTotalLength(0.4, 0.4, 0.4); // 设置轴的长度
    axesActor->SetConeRadius(0.1); // 设置轴锥体的半径
    axesActor->SetCylinderRadius(0.1); // 设置轴圆柱体的半径
    axesActor->SetSphereRadius(0.05); // 设置轴末端的球体半径
    axesActor->SetPosition(0, 0, 0);
    renderer->AddActor(axesActor); // 将坐标器添加到渲染器

    getRenderWindow()->Render(); // 刷新渲染窗口
}

// 创建全局坐标器
void VtkWidget::createAxes()
{
    // 初始化全局坐标系
    axesActor = vtkSmartPointer<vtkAxesActor>::New();

    // 设置 X Y Z 轴标题颜色为黑色
    axesActor->GetXAxisCaptionActor2D()->GetCaptionTextProperty()->SetColor(0.0, 0.0, 0.0);
    axesActor->GetYAxisCaptionActor2D()->GetCaptionTextProperty()->SetColor(0.0, 0.0, 0.0);
    axesActor->GetZAxisCaptionActor2D()->GetCaptionTextProperty()->SetColor(0.0, 0.0, 0.0);

    axesActor->SetTotalLength(0.4, 0.4, 0.4); // 设置轴的长度
    axesActor->SetConeRadius(0.1); // 设置轴锥体的半径
    axesActor->SetCylinderRadius(0.1); // 设置轴圆柱体的半径
    axesActor->SetSphereRadius(0.05); // 设置轴末端的球体半径
    axesActor->SetPosition(0, 0, 0);
    renderer->AddActor(axesActor); // 将坐标器添加到渲染器

    // orientationWidget = vtkSmartPointer<vtkOrientationMarkerWidget>::New();
    // // 设置 X Y Z 轴标题颜色为黑色
    // axes->GetXAxisCaptionActor2D()->GetCaptionTextProperty()->SetColor(0.0, 0.0, 0.0);
    // axes->GetYAxisCaptionActor2D()->GetCaptionTextProperty()->SetColor(0.0, 0.0, 0.0);
    // axes->GetZAxisCaptionActor2D()->GetCaptionTextProperty()->SetColor(0.0, 0.0, 0.0);

    // // 将坐标轴演员添加到orientationWidget
    // orientationWidget->SetOrientationMarker(axes);
    // // 将orientationWidget与交互器关联
    // orientationWidget->SetInteractor(renWin->GetInteractor());
    // // 设置视口
    // orientationWidget->SetViewport(0.0, 0.0, 0.2, 0.2);

    // // orientationWidget->SetEnabled(true);
    // orientationWidget->InteractiveOn();
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
void VtkWidget::ononIsometricView(){
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
    // 清除原先存在的点云的actor
    reDraw();

    // 获取待测量的点云文件map
    auto filemap = m_pMainWin->getpWinFileMgr()->getMeasuredFileMap();

    auto cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // 遍历filemap中所有的文件路径
    for(auto item = filemap.begin();item != filemap.end();item++){
        // 如果文件不隐藏
        if(item.value()){
            pcl::io::loadPCDFile(item.key().toStdString(), *cloud);

            // 将cloud转换为VTK的点集
            vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
            points->SetNumberOfPoints(cloud->points.size());
            for (size_t i = 0; i < cloud->points.size(); ++i)
            {
                points->SetPoint(i, cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
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
            actor->GetProperty()->SetPointSize(3); // 设置点大小
            actor->GetProperty()->SetColor(0.5, 0.5, 0.5);

            renderer->AddActor(actor);
        }
    }

    getRenderWindow()->Render(); // 刷新渲染窗口

}

// 显示完成对比的点云
void VtkWidget::showConvertedCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, const std::string &name){


}

// 比较两个点云的处理函数
void VtkWidget::onCompare()
{
    // 检查点云是否为空
    if (cloud1->empty() || cloud2->empty()) {
        QMessageBox::warning(this, "Warning", "One or both point clouds are empty!");
        return;
    }

    // 创建KD-Tree用于点云2
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud2);

    // 创建一个新的点云对象用于存储比较结果，并设置点云大小
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr comparisonCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
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

    showConvertedCloud(comparisonCloud, "Comparison Cloud");  // 显示比较结果
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
