#ifndef FITTINGLINE_H
#define FITTINGLINE_H

#include <pcl/point_cloud.h>     // PCL 的点云类
#include <pcl/visualization/pcl_visualizer.h> // PCL 的可视化工具

class FittingLine
{
public:
    FittingLine();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr RANSAC(pcl::PointXYZRGB,pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
    bool isPointInLine(const pcl::PointXYZRGB&);
    void setRadius(double);
    void setDistance(double);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getLineCloud();
    Eigen::Vector3f getBegin();
    Eigen::Vector3f getEnd();
private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_subset;//存储searchPoint邻域内的点
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr lineCloud;//存储拟合平面上的点
    pcl::ModelCoefficients::Ptr coefficients;//存储拟合平面方程的4个系数
    pcl::PointIndices::Ptr inliers;//存储内点
    double radius;//邻域
    double distance;//距离阈值

    Eigen::Vector3f normal;//存储直线的法向量
    Eigen::Vector3f onePoint;//存储直线上一点
    Eigen::Vector3f beginPoint;
    Eigen::Vector3f endPoint;

};

#endif // FITTINGLINE_H
