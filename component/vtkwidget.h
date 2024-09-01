#ifndef VTKWIDGET_H
#define VTKWIDGET_H

// 引入 Qt 库的头文件
#include <QWidget>               // QWidget 是所有用户界面对象的基类
#include <QWindow>               // QWindow 提供了窗口的基础功能
#include <QHBoxLayout>           // QHBoxLayout 用于水平布局管理
#include <vtkGenericOpenGLRenderWindow.h> // VTK 的 OpenGL 渲染窗口类
#include <pcl/point_cloud.h>     // PCL 的点云类
#include <pcl/point_types.h>     // PCL 的点类型定义
#include <pcl/io/pcd_io.h>       // PCL 的 PCD 文件输入输出类
#include <pcl/visualization/pcl_visualizer.h> // PCL 的可视化工具

// 定义点的类型
typedef pcl::PointXYZ PointT; // 定义 PointT 为 pcl::PointXYZ 类型
typedef pcl::PointCloud<PointT> PointCloudT; // 定义 PointCloudT 为 pcl::PointCloud<PointT> 类型
typedef pcl::visualization::PCLVisualizer PCLViewer; // 定义 PCLViewer 为 PCLVisualizer 类型
typedef std::shared_ptr<PointCloudT> PointCloudPtr; // 定义 PointCloudPtr 为 PointCloudT 的智能指针

// VtkWidget 类的定义
class VtkWidget : public QWidget
{
    Q_OBJECT

public:
    VtkWidget(QWidget *parent = nullptr);
    ~VtkWidget();

private:
    PointCloudPtr cloudptr; // 点云智能指针
    PCLViewer::Ptr cloud_viewer; // PCL 可视化器的智能指针
};

#endif // VTKWIDGET_H
