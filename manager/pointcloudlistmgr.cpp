#include "pointcloudlistmgr.h"

PointCloudListMgr::PointCloudListMgr() {}

QVector<pcl::PointCloud<pcl::PointXYZRGB>> &PointCloudListMgr::getProductCloudList()
{
    return productCloudList;
}

std::unordered_map<QString, pcl::PointCloud<pcl::PointXYZRGB>> &PointCloudListMgr::getPointCloudList()
{
    return pointCloudList;
}

void PointCloudListMgr::CreateCloudFromFile(QString str)
{
    // 创建并加载RGB点云
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::io::loadPCDFile(str.toStdString(), cloud);

    // 将新的点云加入容器
    getPointCloudList()[str] = cloud;
}
