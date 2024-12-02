#ifndef FITTINGSPHERE_H
#define FITTINGSPHERE_H

#include <pcl/point_cloud.h>     // PCL 的点云类
#include <pcl/visualization/pcl_visualizer.h> // PCL 的可视化工具

class FittingSphere
{
public:
    FittingSphere();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr RANSAC(pcl::PointXYZRGB,pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
    bool isPointInSphere(const pcl::PointXYZRGB&);
    void setRadius(double);
    void setDistance(double);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getSphereCloud();
    Eigen::Vector3f getCenter();
    double getRad();

private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_subset;//存储searchPoint邻域内的点
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sphereCloud;//存储拟合球上的点
    pcl::ModelCoefficients::Ptr coefficients;//存储拟合球方程的系数
    pcl::PointIndices::Ptr inliers;//存储内点
    double radius;//邻域
    double distance;//距离阈值

    Eigen::Vector3f center;//球中心
    double r;//球半径
};

#endif // FITTINGSPHERE_H
