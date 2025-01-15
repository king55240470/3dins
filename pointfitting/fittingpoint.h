#ifndef FITTINGPOINT_H
#define FITTINGPOINT_H

#include <pcl/point_cloud.h>     // PCL 的点云类
#include <pcl/visualization/pcl_visualizer.h> // PCL 的可视化工具

#include <QVector4D>

class FittingPoint
{
public:
    FittingPoint();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr RANSAC(pcl::PointXYZRGB,pcl::PointCloud<pcl::PointXYZRGB>::Ptr);
    QVector4D getPoint(){
        return point;
    }
private:
    QVector4D point;//存储最近点
};

#endif // FITTINGPOINT_H
