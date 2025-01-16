#ifndef FITTINGCIRCLE_H
#define FITTINGCIRCLE_H

#include <pcl/point_cloud.h>     // PCL 的点云类
#include <pcl/visualization/pcl_visualizer.h> // PCL 的可视化工具

class FittingCircle
{
public:
    FittingCircle();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr RANSAC(pcl::PointXYZRGB,pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
    bool isPointInCircle(const pcl::PointXYZRGB&);
    void setRadius(double);
    void setDistance(double);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCircleCloud();
    Eigen::Vector3f getCenter();
    double getRad();
    Eigen::Vector3f getNormal();
private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_subset;//存储searchPoint邻域内的点
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr circleCloud;//存储拟合圆上的点
    pcl::ModelCoefficients::Ptr coefficients;//存储拟合圆方程的系数
    pcl::PointIndices::Ptr inliers;//存储内点
    double radius;//邻域
    double distance;//距离阈值

    Eigen::Vector3f center;//圆中心
    double r;//圆半径
    Eigen::Vector3f normal;//圆法向量
};

#endif // FITTINGCIRCLE_H
