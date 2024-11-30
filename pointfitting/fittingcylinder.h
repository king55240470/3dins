#ifndef FITTINGCYLINDER_H
#define FITTINGCYLINDER_H

#include <pcl/point_cloud.h>     // PCL 的点云类
#include <pcl/visualization/pcl_visualizer.h> // PCL 的可视化工具

class FittingCylinder
{
public:
    FittingCylinder();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr RANSAC(pcl::PointXYZRGB,pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
    bool isPointInCylinder(const pcl::PointXYZRGB&);
    void setRadius(double);
    void setDistance(double);
    Eigen::Vector3f getNormal();
    Eigen::Vector3f getCenter();
    double getDiameter();
    double getHeight();
private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_subset;//存储searchPoint邻域内的点
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cylinderCloud;//存储拟合圆柱上的点
    pcl::ModelCoefficients::Ptr coefficients;//存储拟合圆柱方程的7个系数
    pcl::PointIndices::Ptr inliers;//存储内点
    double radius;//邻域
    double distance;//距离阈值

    Eigen::Vector3f normal;//圆柱法向量
    Eigen::Vector3f center;//圆柱中心
    //Eigen::Vector3f bottomCenter;//圆柱底面圆心中心
    double diameter;//圆柱直径
    double height;//圆柱高度
};

#endif // FITTINGCYLINDER_H
