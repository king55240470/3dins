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


// 定义并初始化渲染器、渲染窗口和交互器
vtkSmartPointer<vtkRenderer> VtkWidget::m_renderer = vtkSmartPointer<vtkRenderer>::New();
vtkSmartPointer<vtkRenderWindow> VtkWidget::m_renWin = vtkSmartPointer<vtkRenderWindow>::New();
vtkSmartPointer<vtkRenderWindowInteractor> VtkWidget::m_interactor= vtkSmartPointer<vtkRenderWindowInteractor>::New();

// 渲染窗口大小
#define WIDTH 1000
#define HEIGHT 800

VtkWidget::VtkWidget(QWidget *parent)
    : QWidget(parent)
{
    m_pMainWin = (MainWindow*) parent;

    // 为交互器设置窗口
    m_interactor->SetRenderWindow(m_renWin);

    // 创建球体源
    vtkSmartPointer<vtkSphereSource> sphere = vtkSmartPointer<vtkSphereSource>::New();
    sphere->SetCenter(0, 2, 0);
    sphere->SetRadius(1);
    sphere->SetEndTheta(360);
    sphere->SetThetaResolution(50);

    // 创建映射器
    auto mapper_2 = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper_2->SetInputConnection(sphere->GetOutputPort());

    // 创建执行器
    auto actor_2 = vtkSmartPointer<vtkActor>::New();
    actor_2->SetMapper(mapper_2);
    actor_2->GetProperty()->SetColor(0.7, 0.3, 0.3);

    // 创建坐标器
    auto axes = vtkSmartPointer<vtkAxesActor>::New();
    axes->SetAxisLabels(true);
    axes->SetAxisLabels(true);
    axes->SetAxisLabels(true);
    axes->SetTotalLength(0.7, 0.7, 0.7); // 设置轴的长度
    axes->SetConeRadius(0.03); // 设置轴锥体的半径
    axes->SetCylinderRadius(0.02); // 设置轴圆柱体的半径
    axes->SetSphereRadius(0.03); // 设置轴末端的球体半径
    axes->SetPosition(-WIDTH * 0.9, -HEIGHT * 0.9, 0.0);

    // 将坐标器添加到渲染器中
    m_renderer->AddActor(axes);

    m_renderer->SetBackground(0.1, 0.2, 0.4);
    // m_renderer->AddActor(actor_2); // 将球体添加到渲染器
    m_renWin->AddRenderer(m_renderer);  // 将渲染器添加到渲染窗口
    m_renWin->SetSize(WIDTH, HEIGHT);

    // 设置 VTK 渲染窗口到 QWidget
    QVTKOpenGLNativeWidget* vtkWidget = new QVTKOpenGLNativeWidget(this);
    vtkWidget->setRenderWindow(m_renWin);
    QHBoxLayout *layout = new QHBoxLayout(this);
    layout->addWidget(vtkWidget);
    setLayout(layout);

    m_renWin->Render(); // 开始渲染


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
    return m_renWin;
}

vtkSmartPointer<vtkRenderer> VtkWidget::getRenderer(){
    return m_renderer;
}

void VtkWidget::addActor(vtkSmartPointer<vtkActor>& actor){
    m_renderer->AddActor(actor);
}

void VtkWidget::UpdateInfo(){
    reDraw();
}


void VtkWidget::reDraw(){
    // 清除渲染窗口里所有actor
    VtkWidget::getRenderer()->Clear();

    // 遍历m_entityList重新绘制
    for(auto& entity : m_pMainWin->m_EntityListMgr->getEntityList()){
        entity->draw();
    }

    // 将新的渲染器加入渲染窗口
    VtkWidget::getRenderWindow()->AddRenderer(VtkWidget::getRenderer());
    getRenderWindow()->Render();
}

VtkWidget::~VtkWidget() {}
