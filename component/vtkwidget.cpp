#include "vtkwidget.h"

#include <QVTKOpenGLNativeWidget.h>
#include <vtkSphereSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkAxesActor.h>
#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingVolumeOpenGL2);
VTK_MODULE_INIT(vtkRenderingFreeType);

// 渲染窗口大小
#define WIDTH 1000
#define HEIGHT 800

VtkWidget::VtkWidget(QWidget *parent)
    : QWidget(parent)
{
    m_pMainWin = (MainWindow*) parent;


    renWin = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    renderer = vtkSmartPointer<vtkRenderer>::New();

    // 创建坐标器
    auto axes = vtkSmartPointer<vtkAxesActor>::New();
    axes->SetAxisLabels(true);
    axes->SetAxisLabels(true);
    axes->SetAxisLabels(true);
    axes->SetTotalLength(0.2, 0.2, 0.2); // 设置轴的长度
    axes->SetConeRadius(0.03); // 设置轴锥体的半径
    axes->SetCylinderRadius(0.02); // 设置轴圆柱体的半径
    axes->SetSphereRadius(0.03); // 设置轴末端的球体半径
    axes->SetPosition(0, 0, 0);

    // 将坐标器添加到渲染器中
    renderer->SetBackground(255, 255, 255);
    renderer->AddActor(axes);

    renWin->AddRenderer(renderer);  // 将渲染器添加到渲染窗口

    // 设置 VTK 渲染窗口到 QWidget
    QHBoxLayout *mainlayout = new QHBoxLayout;
    setUpVtk(mainlayout); // 配置vtk窗口
    this->setLayout(mainlayout);


    // // 创建 PCLViewer 对象，用于显示点云
    // cloud_viewer.reset(new PCLViewer("Viewer"));
    // cloud_viewer->setShowFPS(false);  // 不显示帧率
    // cloud_viewer->setBackgroundColor(0.1, 0.2, 0.4);
    // // 获取渲染窗口的窗口 ID，并将其转换为 QWindow 对象
    // auto viewerWinId = QWindow::fromWinId((WId)cloud_viewer->getRenderWindow()->GetGenericWindowId());
    // // 创建一个窗口容器，将 PCLViewer 的窗口嵌入到 Qt 窗口中
    // QWidget *widget = QWidget::createWindowContainer(viewerWinId, nullptr);
    // // 创建一个垂直布局
    // QVBoxLayout* mainLayout = new QVBoxLayout;
    // mainLayout->addWidget(widget);  // 将窗口容器添加到布局中
    // // 设置主窗口的中心部件的布局
    // setLayout(mainLayout);
    // // 创建一个点云智能指针
    // cloudptr.reset(new PointCloudT);
    // // 从指定路径加载 PCD 文件到点云对象中
    // // pcl::io::loadPCDFile("D:\\Lenovo\\Acun\\3din\\bunny.pcd", *cloudptr);
    // // pcl::io::loadPCDFile("E:\\pcl\\maize.pcd", *cloudptr);
    // pcl::io::loadPCDFile("E:\\pcl\\bunny.pcd", *cloudptr);
    // // 定义颜色处理的轴
    // const std::string axis ="z";
    // // 创建颜色处理器，基于指定的轴为点云着色
    // pcl::visualization::PointCloudColorHandlerGenericField<PointT> color_handler(cloudptr, axis);
    // // 将点云添加到 PCLViewer 中
    // cloud_viewer->addPointCloud(cloudptr, color_handler, "cloud");

}

vtkSmartPointer<vtkRenderWindow> VtkWidget::getRenderWindow(){
    return renWin;
}

vtkSmartPointer<vtkRenderer>& VtkWidget::getRenderer(){
    // return m_renderer;
    return renderer;
}

void VtkWidget::addActor(vtkSmartPointer<vtkActor> actor){
    renderer->AddActor(actor);
}

void VtkWidget::UpdateInfo(){
    reDraw();
}

void VtkWidget::reDraw(){
    // 清除渲染器中的所有 actor
    auto* actorCollection = getRenderer()->GetActors();

    // 创建一个迭代器用于遍历actor集合
    vtkCollectionSimpleIterator it;

    // 初始化迭代器，准备遍历actor集合
    actorCollection->InitTraversal(it);

    vtkSmartPointer<vtkActor> actor;
    while ((actor = actorCollection->GetNextActor(it)) != nullptr)
    {
        renderer->RemoveActor(actor);
    }

    // 遍历m_entityList重新绘制
    for(auto& entity : m_pMainWin->m_EntityListMgr->getEntityList()){
        addActor(entity->draw());
    }

    getRenderWindow()->Render(); // 刷新渲染窗口
}

VtkWidget::~VtkWidget() {}

void VtkWidget::setUpVtk(QHBoxLayout *layout){
    QVTKOpenGLNativeWidget* vtkWidget = new QVTKOpenGLNativeWidget(this);
    vtkWidget->setRenderWindow(renWin);
    layout->addWidget(vtkWidget);
    renWin->Render();
}
