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
    QVector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& getPclList();

private:
    QVector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> pointCloudList; // 存储所有打开的点云

};

#endif // POINTCLOUDLISTMGR_H
