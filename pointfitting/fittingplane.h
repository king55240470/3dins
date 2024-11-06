#ifndef FITTINGPLANE_H
#define FITTINGPLANE_H

#include <pcl/point_cloud.h>     // PCL 的点云类
#include <pcl/visualization/pcl_visualizer.h> // PCL 的可视化工具
#include <QDialog>
#include <QGridLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>

class FittingPlane
{
public:
    FittingPlane();
    void RANSAC(pcl::PointXYZ,pcl::PointCloud<pcl::PointXYZ>::Ptr);
    bool isPointInPlane(const pcl::PointXYZ&);
    void visualizePlane();
    void setDis();//设置领域和距离阈值的对话框
    void onBtnClick();
private:
    // pcl::PointXYZ searchPoint;//实现拟合平面的点
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloudptr;//存储打开的点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_subset;//存储searchPoint邻域内的点
    pcl::PointCloud<pcl::PointXYZ>::Ptr planeCloud;//存储拟合平面上的点
    pcl::ModelCoefficients::Ptr coefficients;//存储拟合平面方程的4个系数
    pcl::PointIndices::Ptr inliers;//存储内点
    double radious;
    int distance;

    //对话框
    QDialog *dialog;
    QGridLayout *layout;
    QLabel *lab1;
    QLabel *lab2;
    QLineEdit *rad;
    QLineEdit *dis;
    QPushButton *btn;
};

#endif // FITTINGPLANE_H
