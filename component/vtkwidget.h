#ifndef VTKWIDGET_H
#define VTKWIDGET_H

#include <QWidget>
#include <QWindow>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPushButton>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>

#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkGenericRenderWindowInteractor.h>

#include <vtkActor.h>
#include <vtkSmartPointer.h>

#include <pcl/point_cloud.h>     // PCL 的点云类
#include <pcl/point_types.h>     // PCL 的点类型定义
#include <pcl/io/pcd_io.h>       // PCL 的 PCD 文件输入输出类
#include <pcl/visualization/pcl_visualizer.h> // PCL 的可视化工具
#include "mainwindow.h"

#include <QVTKOpenGLNativeWidget.h>
#include <vtkSphereSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkProperty.h>
#include <vtkCamera.h>
#include <vtkAxesActor.h>
#include <vtkCubeAxesActor.h>
#include <vtkCaptionActor2D.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingVolumeOpenGL2);
VTK_MODULE_INIT(vtkRenderingFreeType);

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

    // 配置vtk窗口
    void setUpVtk(QVBoxLayout *layout);

    // 获取m_renWin
    vtkSmartPointer<vtkRenderWindow> getRenderWindow();

    // 获取渲染器
    vtkSmartPointer<vtkRenderer>& getRenderer();

    // 将draw返回的actor添加到渲染器中
    void addActor(vtkSmartPointer<vtkActor> actor);

    // 在notifsubscribey里更新信息
    void UpdateInfo();

    // 当添加新的元素后，遍历m_entityList重新绘制
    void reDraw();

    // 创建左下角坐标轴
    void createAxes();

private:
    PointCloudPtr cloudptr; // 点云智能指针
    PCLViewer::Ptr cloud_viewer; // PCL 可视化器的智能指针

    QVTKOpenGLNativeWidget* vtkWidget; // vtk窗口

    // 创建渲染器、渲染窗口和交互器
    vtkSmartPointer<vtkRenderer> renderer;
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> renWin;
    vtkSmartPointer<vtkRenderWindowInteractor> interactor;

    // 创建坐标器
    vtkSmartPointer<vtkAxesActor> axesActor;
    // 创建交互部件来封装坐标器
    vtkSmartPointer<vtkOrientationMarkerWidget> orientationWidget;

    MainWindow *m_pMainWin=nullptr;
signals:


private slots:
    void onTopViewClicked(); // 相机视角1
    void onRightViewClicked(); // 相机视角2
    void onFrontViewClicked(); // 相机视角3
};

#endif // VTKWIDGET_H
