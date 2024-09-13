#ifndef VTKWIDGET_H
#define VTKWIDGET_H
#include "manager/centitymgr.h"

#include <QWidget>               // QWidget 是所有用户界面对象的基类
#include <QWindow>               // QWindow 提供了窗口的基础功能
#include <QHBoxLayout>           // QHBoxLayout 用于水平布局管理
#include <vtkGenericOpenGLRenderWindow.h> // VTK 的 OpenGL 渲染窗口类
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkActor.h>
#include <vtkSmartPointer.h>
#include <vtkGenericRenderWindowInteractor.h>
#include <vtkGenericOpenGLRenderWindow.h>

#include <pcl/point_cloud.h>     // PCL 的点云类
#include <pcl/point_types.h>     // PCL 的点类型定义
#include <pcl/io/pcd_io.h>       // PCL 的 PCD 文件输入输出类
#include <pcl/visualization/pcl_visualizer.h> // PCL 的可视化工具

// 定义点的类型
typedef pcl::PointXYZ PointT; // 定义 PointT 为 pcl::PointXYZ 类型
typedef pcl::PointCloud<PointT> PointCloudT; // 定义 PointCloudT 为 pcl::PointCloud<PointT> 类型
typedef pcl::visualization::PCLVisualizer PCLViewer; // 定义 PCLViewer 为 PCLVisualizer 类型
typedef std::shared_ptr<PointCloudT> PointCloudPtr; // 定义 PointCloudPtr 为 PointCloudT 的智能指针

class VtkWidget : public QWidget
{
    Q_OBJECT

public:
    VtkWidget(QWidget *parent = nullptr);
    ~VtkWidget();

    // 获取m_renWin
    vtkSmartPointer<vtkRenderWindow> getRenderWindow();

    // 获取渲染器
    static vtkSmartPointer<vtkRenderer> getRenderer();

    // 将draw返回的actor添加到渲染器中
    static void addActor(vtkSmartPointer<vtkActor>& actor);

    // 实现在notify里更新信息
    void UpdateInfo();
private:
    PointCloudPtr cloudptr; // 点云智能指针
    PCLViewer::Ptr cloud_viewer; // PCL 可视化器的智能指针

    // 为所有的actor创建渲染器、渲染窗口和交互器
    static vtkSmartPointer<vtkRenderer> m_renderer;
    static vtkSmartPointer<vtkGenericOpenGLRenderWindow> m_renWin;
    static vtkSmartPointer<vtkGenericRenderWindowInteractor> m_interactor;

};

#endif // VTKWIDGET_H
