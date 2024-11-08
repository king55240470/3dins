#ifndef POINTCLOUDLISTMGR_H
#define POINTCLOUDLISTMGR_H

#include <QVector>

#include <pcl/common/common.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/distances.h>  // PCL距离计算函数

class PointCloudListMgr
{
public:
    PointCloudListMgr();
    std::unordered_map<QString, pcl::PointCloud<pcl::PointXYZRGB>> &getPointCloudList();
    QVector<pcl::PointCloud<pcl::PointXYZRGB>>& getProductCloudList();

    void CreateCloudFromFile(QString str);// 给打开的文件分配点云

private:
    // 存储所有打开的点云和对应的文件路径
    std::unordered_map<QString, pcl::PointCloud<pcl::PointXYZRGB>> pointCloudList;

    // 存储所有生成的点云，通过对比和拟合等
    QVector<pcl::PointCloud<pcl::PointXYZRGB>> productCloudList;

};

#endif // POINTCLOUDLISTMGR_H
