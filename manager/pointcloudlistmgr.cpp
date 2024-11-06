#include "pointcloudlistmgr.h"

PointCloudListMgr::PointCloudListMgr() {}

QVector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &PointCloudListMgr::getPclList()
{
    return pointCloudList;
}
