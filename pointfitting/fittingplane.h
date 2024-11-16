#ifndef FITTINGPLANE_H
#define FITTINGPLANE_H

#include <pcl/point_cloud.h>     // PCL 的点云类
#include <pcl/visualization/pcl_visualizer.h> // PCL 的可视化工具

class setDataWidget;
class FittingPlane
{
public:
    FittingPlane();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr RANSAC(pcl::PointXYZRGB,pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
    bool isPointInPlane(const pcl::PointXYZRGB&);
    void setRadious(double);
    void setDistance(double);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getPlaneCloud();
    Eigen::Vector3f getNormal();
    Eigen::Vector4f getCenter();
    double getLength();
    double getWidth();
    Eigen::Vector3f getLength_Direction();
private:
    // pcl::PointXYZRGB searchPoint;//实现拟合平面的点
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudptr;//存储打开的点云
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_subset;//存储searchPoint邻域内的点
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr planeCloud;//存储拟合平面上的点
    pcl::ModelCoefficients::Ptr coefficients;//存储拟合平面方程的4个系数
    pcl::PointIndices::Ptr inliers;//存储内点
    double radious;
    double distance;
    Eigen::Vector3f normal;//存储平面的法向量
    Eigen::Vector4f center;//存储平面中心
    double length;//平面的长
    double width;//平面的宽
    Eigen::Vector3f length_direction;//长边的方向向量
};

#endif // FITTINGPLANE_H
