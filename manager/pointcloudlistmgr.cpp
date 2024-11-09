#include "pointcloudlistmgr.h"

PointCloudListMgr::PointCloudListMgr() {}

QVector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &PointCloudListMgr::getProductCloudList()
{
    return productCloudList;
}

QMap<QString, pcl::PointCloud<pcl::PointXYZRGB>> &PointCloudListMgr::getPointCloudList()
{
    return pointCloudList;
}

void PointCloudListMgr::CreateCloudFromFile(QString str)
{
    // 创建并加载RGB点云
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::io::loadPCDFile(str.toStdString(), cloud);

    // 将新的点云加入容器
    getPointCloudList().insert(str, cloud);
}
