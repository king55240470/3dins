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
#include <vtkTextProperty.h>
#include <vtkAutoInit.h>
#include <vtkCommand.h>
#include <vtkPlaneSource.h>
#include <vtkActor2D.h>
#include <vtkPolyDataMapper2D.h>
#include <vtkNamedColors.h>
#include <qlabel.h>
#include <vtkImageActor.h>
#include <vtkPNGReader.h>
#include <vtkImageMapper.h>
#include <vtkJPEGReader.h>.h>
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

    void setUpVtk(QVBoxLayout *layout);// 配置vtk窗口
    vtkSmartPointer<vtkRenderWindow> getRenderWindow();// 获取m_renWin
    vtkSmartPointer<vtkRenderer>& getRenderer();// 获取渲染器

    void UpdateInfo();// 在notifsubscribey里更新信息
    void reDrawCentity();// 重新绘制基本图形和坐标轴
    void reDrawCloud();// 重新绘制点云

    void createAxes();// 创建左下角坐标轴

    // 切换相机视角
    void onTopView(); // 俯视
    void onRightView(); // 右侧
    void onFrontView(); // 正视
    void onIsometricView(); // 旋转立体

    void showConvertedCloud();// 将点云转为vtk的顶点图形并显示
    void onCompare();// 比较两个点云
    void onAlign();    // 配准的函数

    // 显示选中的图形的信息
    void setCentity(CEntity*entity);  //传入centity对象
    void MouseDrag();
    void OnMouseMove();
    void OnLeftButtonPress();
    void OnLeftButtonRelease();
    void createText();
    void createTextBox();
    void createLine();
    void Linechange();
    void closeText();
    void GetScreenCoordinates(vtkRenderer* renderer, double pt[3], double screenCoord[2]);
private:
    QVTKOpenGLNativeWidget* vtkWidget; // vtk窗口
    MainWindow *m_pMainWin = nullptr; // mainwindow指针

    // 创建渲染器、渲染窗口和交互器
    vtkSmartPointer<vtkRenderer> renderer;
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> renWin;

    pcl::PointCloud<pcl::PointXYZRGB> tempCloud; // 转换rgb点云用的临时点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1; // 对比用的两个点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr comparisonCloud;

    vtkSmartPointer<vtkTextActor> infoTextActor;// 浮动信息文本演员
    vtkSmartPointer<vtkActor2D> rectangleActor; // 背景和边框
    vtkSmartPointer<vtkActor2D> lineActor;//指向线条
    vtkSmartPointer<vtkPNGReader> pngReader; //储存图片信息
    vtkSmartPointer<vtkImageActor> iconActor; //图片演员
    vtkSmartPointer<vtkOrientationMarkerWidget> axeWidget; // 创建窗口部件来封装坐标器
    vtkSmartPointer<vtkOrientationMarkerWidget> textWidget; // 浮动窗口，用于显示信息
    CEntity* elementEntity;//储存传入的entity
    vtkTimeStamp lastLeftClickTime; // 时间戳，记录上次左键点击的时间
    // double lastLeftClickPos[2]; // 记录左键按下时的位置
    bool isDragging=false;  //判断注释是否能移动
    CPosition b;//储存指向箭头的终点
    vtkSmartPointer<vtkPolyData> linePolyData;//储存线的data
    vtkSmartPointer<vtkPolyDataMapper2D> lineMapper;//指向线的mapper
    vtkSmartPointer<MouseInteractorHighlightActor>m_highlightstyle;
public slots:

};

#endif // VTKWIDGET_H
