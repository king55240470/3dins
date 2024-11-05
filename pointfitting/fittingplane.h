#ifndef FITTINGPLANE_H
#define FITTINGPLANE_H

#include <pcl/point_cloud.h>     // PCL 的点云类
#include <pcl/visualization/pcl_visualizer.h> // PCL 的可视化工具

//定义点的类型（为Vtk中的类型名创建别名）
typedef pcl::PointXYZ PointT; // 定义 PointT 为 pcl::PointXYZ 类型
typedef pcl::PointCloud<PointT> PointCloudT; // 定义 PointCloudT 为 pcl::PointCloud<PointT> 类型
typedef std::shared_ptr<PointCloudT> PointCloudPtr;// 定义 PointCloudPtr 为 PointCloudT

typedef pcl::visualization::PCLVisualizer PCLViewer; // 定义 PCLViewer 为 PCLVisualizer 类型

class FittingPlane
{
public:
    FittingPlane();
    void RANSAC();
    bool isPointInPlane(const PointT&);
    void visualizePlane();
private:
    PointT searchPoint;//实现拟合平面的点
    PointCloudPtr cloudptr;//点云智能指针
    PointCloudPtr cloud_subset;//存储searchPoint邻域内的点
    PointCloudPtr planeCloud;//存储拟合平面上的点
    pcl::ModelCoefficients::Ptr coefficients;//存储拟合平面方程的4个系数
    pcl::PointIndices::Ptr inliers;//存储内点
};

#endif // FITTINGPLANE_H
