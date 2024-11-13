#ifndef POINTCLOUDLISTMGR_H
#define POINTCLOUDLISTMGR_H

#include "geometry/centitytypes.h"
#include "pointfitting/fittingplane.h"

#include <QVector>
#include <QMap>
#include <pcl/common/common.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/distances.h>  // PCL距离计算函数

class PointCloudListMgr
{
public:
    PointCloudListMgr();
    QMap<QString, pcl::PointCloud<pcl::PointXYZRGB>> &getPointCloudList();
    QVector<pcl::PointCloud<pcl::PointXYZRGB>>& getProductCloudList();
    CPointCloud* CreateCloudFromFile(QString str);// 给打开的文件分配点云
    CPointCloud* CreateFittingCloud(pcl::PointCloud<pcl::PointXYZRGB> plane);// 创建不同的点云centity对象

    pcl::PointCloud<pcl::PointXYZRGB> getTempCloud(){
        return cloud;
    };
private:
    // 存储所有打开的点云和对应的文件路径
    QMap<QString, pcl::PointCloud<pcl::PointXYZRGB>> pointCloudList;

    // 存储所有生成的点云，通过对比和拟合等
    QVector<pcl::PointCloud<pcl::PointXYZRGB>> productCloudList;

    pcl::PointCloud<pcl::PointXYZRGB> cloud; // 临时点云对象
};

#endif // POINTCLOUDLISTMGR_H
