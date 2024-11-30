#ifndef FITTINGCONE_H
#define FITTINGCONE_H

#include <pcl/point_cloud.h>     // PCL 的点云类
#include <pcl/visualization/pcl_visualizer.h> // PCL 的可视化工具

class FittingCone
{
public:
    FittingCone();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr RANSAC(pcl::PointXYZRGB,pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
    bool isPointInCone(const pcl::PointXYZRGB&);
    void setRadius(double);
    void setDistance(double);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getConeCloud();
    Eigen::Vector3f getTopCenter();
    Eigen::Vector3f getNormal();
    double getAngle();
    double getHeight();
private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_subset;//存储searchPoint邻域内的点
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr coneCloud;//存储拟合圆锥上的点
    pcl::ModelCoefficients::Ptr coefficients;//存储拟合圆锥方程的系数
    pcl::PointIndices::Ptr inliers;//存储内点
    double radius;//邻域
    double distance;//距离阈值

    Eigen::Vector3f topCenter;//圆锥顶点
    Eigen::Vector3f normal;//圆锥轴向量
    double angle;//圆锥张开角度
    double height;//圆锥高度
};

#endif // FITTINGCONE_H
