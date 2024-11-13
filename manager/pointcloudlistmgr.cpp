#include "pointcloudlistmgr.h"

PointCloudListMgr::PointCloudListMgr() {}

QVector<pcl::PointCloud<pcl::PointXYZRGB>> &PointCloudListMgr::getProductCloudList()
{
    return productCloudList;
}

QMap<QString, pcl::PointCloud<pcl::PointXYZRGB>> &PointCloudListMgr::getPointCloudList()
{
    return pointCloudList;
}
CPointCloud* PointCloudListMgr::CreateCloudFromFile(QString str)
{
    // 创建并加载RGB点云
    pcl::io::loadPCDFile(str.toStdString(), cloud);

    // 创建新的点云实体
    CPointCloud* pointCloud=new CPointCloud();
    pointCloud->isFileCloud = true;
    pointCloud->setPointCloud(cloud);

    // 将新的点云加入容器
    // getPointCloudList().insert(str, cloud);
    return pointCloud;
}

CPointCloud *PointCloudListMgr::CreateFittingCloud(pcl::PointCloud<pcl::PointXYZRGB> plane)
{
    CPointCloud* pointCloud=new CPointCloud();
    pointCloud->isFittingCloud = true; // 拟合出的点云
    pointCloud->setPointCloud(plane);

    return pointCloud;
}
