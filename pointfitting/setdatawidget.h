#ifndef SETDATAWIDGET_H
#define SETDATAWIDGET_H

#include <pcl/point_cloud.h>     // PCL 的点云类
#include <pcl/visualization/pcl_visualizer.h> // PCL 的可视化工具

#include <QWidget>
#include <QDialog>
#include <QGridLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>

class MainWindow;
class FittingPlane;
class FittingCylinder;
class setDataWidget : public QWidget
{
    Q_OBJECT
public:
    explicit setDataWidget(QWidget *parent = nullptr);

    void setPlaneData(pcl::PointXYZRGB,pcl::PointCloud<pcl::PointXYZRGB>::Ptr);//设置拟合平面的领域和距离阈值的对话框
    void PlaneBtnClick();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getFittingPlane();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getPlaneCloud();
    FittingPlane *getPlane();

    void setCylinderData(pcl::PointXYZRGB,pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
    void CylinderBtnClick();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getFittingCylinder();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCylinderCloud();
    FittingCylinder *getCylinder();
private:
    MainWindow *m_pMainWin;

    pcl::PointXYZRGB point;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudptr;

    //对话框
    QDialog *dialog;
    QGridLayout *layout;
    QLabel *lab1;
    QLabel *lab2;
    QLineEdit *rad;
    QLineEdit *dis;
    QPushButton *btn;

    //拟合平面
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr fittingPlane;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr planeCloud;
    FittingPlane *plane;

    //拟合圆柱
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr fittingCylinder;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cylinderCloud;
    FittingCylinder *cylinder;

signals:
};

#endif // SETDATAWIDGET_H
