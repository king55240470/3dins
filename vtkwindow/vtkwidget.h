#ifndef VTKWIDGET_H
#define VTKWIDGET_H

#include "mainwindow.h"
#include "clickhighlightstyle.h"
#include "custominteractorstyle.h"
#include "manager/filemgr.h"
#include "component/toolwidget.h"
#include <QWidget>
#include <QVTKOpenGLWidget.h>
#include <QVTKOpenGLNativeWidget.h>
#include <QWindow>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPushButton>
#include <QImage>
#include <QTimer>
#include <QPixmap>

#include <pcl/common/common.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <cmath>
#include <limits>
#include <pcl/common/distances.h>  // PCL距离计算函数
#include <pcl/visualization/pcl_visualizer.h>  // PCL可视化库
#include <pcl/registration/icp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/icp.h>

#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkGenericRenderWindowInteractor.h>
#include <vtkActor.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkProp3D.h>
#include <vtkProperty.h>
#include <vtkCamera.h>
#include <vtkAxesActor.h>
#include <vtkCubeAxesActor.h>
#include <vtkCaptionActor2D.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkPropPicker.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkCallbackCommand.h>
#include <vtkGlyphSource2D.h>
#include <vtkTextActor.h>
#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingVolumeOpenGL2);
VTK_MODULE_INIT(vtkRenderingFreeType);

class VtkWidget : public QWidget
{
    Q_OBJECT

public:
    VtkWidget(QWidget *parent = nullptr);
    ~VtkWidget() {};

    // 配置vtk窗口
    void setUpVtk(QVBoxLayout *layout);
    // void setUpVtk();

    // 配置点云文件
    void setUpPcl();

    // 获取m_renWin
    vtkSmartPointer<vtkRenderWindow> getRenderWindow();

    // 获取渲染器和交互器
    vtkSmartPointer<vtkRenderer>& getRenderer();

    // 在notifsubscribey里更新信息
    void UpdateInfo();

    // 当添加新的元素后，遍历m_entityList重新绘制
    void reDraw();

    // 创建左下角坐标轴
    void createAxes();

    // 切换相机视角
    void onTopView(); // 俯视
    void onRightView(); // 右侧
    void onFrontView(); // 正视
    void onIsometricView(); // 旋转立体视角

    // 将点云转为vtk的顶点图形并显示
    void showConvertedCloud();

    // 比较两个点云
    void onCompare();

    // 用于显示对比完成的两个点云
    void VtkWidget::showConvertedCloud(pcl::PointCloud<pcl::PointXYZRGB> cloud_rgb_1);

private:
    QVTKOpenGLNativeWidget* vtkWidget; // vtk窗口
    MainWindow *m_pMainWin = nullptr; // mainwindow指针

    // 创建渲染器、渲染窗口和交互器
    vtkSmartPointer<vtkRenderer> renderer;
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> renWin;

    // 创建交互部件来封装坐标器
    vtkSmartPointer<vtkOrientationMarkerWidget> orientationWidget;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1; // 基准点云1
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr comparisonCloud;

private slots:
    // 配准的函数
    void onAlign();
};

#endif // VTKWIDGET_H
