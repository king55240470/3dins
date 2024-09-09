#include "vtkwidget.h"

VtkWidget::VtkWidget(QWidget *parent)
    : QWidget(parent)
{
    setWindowTitle("PCL Test");  // 设置窗口标题

    // 创建 PCLViewer 对象，用于显示点云
    cloud_viewer.reset(new PCLViewer("Viewer"));
    cloud_viewer->setShowFPS(false);  // 不显示帧率

    // 获取渲染窗口的窗口 ID，并将其转换为 QWindow 对象
    auto viewerWinId = QWindow::fromWinId((WId)cloud_viewer->getRenderWindow()->GetGenericWindowId());

    // 创建一个窗口容器，将 PCLViewer 的窗口嵌入到 Qt 窗口中
    QWidget *widget = QWidget::createWindowContainer(viewerWinId, nullptr);

    // 创建一个垂直布局
    QVBoxLayout* mainLayout = new QVBoxLayout;
    mainLayout->addWidget(widget);  // 将窗口容器添加到布局中

    // 设置主窗口的中心部件的布局
    setLayout(mainLayout);

    // 创建一个点云智能指针
    cloudptr.reset(new PointCloudT);

    // 从指定路径加载 PCD 文件到点云对象中

    pcl::io::loadPCDFile("C:/Users/Lenovo/Desktop/computerprogram/PCL/pcl/bunny.pcd", *cloudptr);

    pcl::io::loadPCDFile("D:\\Lenovo\\Acun\\3din\\bunny.pcd", *cloudptr);

    pcl::io::loadPCDFile("E:\\pcl\\bunny.pcd", *cloudptr);

    // 定义颜色处理的轴
    const std::string axis ="z";

    // 创建颜色处理器，基于指定的轴为点云着色
    pcl::visualization::PointCloudColorHandlerGenericField<PointT> color_handler(cloudptr, axis);

    // 将点云添加到 PCLViewer 中
    cloud_viewer->addPointCloud(cloudptr, color_handler, "cloud");

}

VtkWidget::~VtkWidget()
{
}
