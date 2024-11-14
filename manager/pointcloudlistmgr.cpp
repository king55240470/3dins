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
    QString suffix = str.section('.', -1, -1); // 从最后一个'.'到最后一个字符，获取后缀名

    // 创建并加载RGB点云
    if(suffix == "pcd")
        pcl::io::loadPCDFile(str.toStdString(), tempCloud);
    else
        pcl::io::loadPLYFile(str.toStdString(), tempCloud);

    // 创建新的点云实体
    CPointCloud* pointCloud=new CPointCloud();
    pointCloud->isFileCloud = true;
    pointCloud->setPointCloud(tempCloud);

    // 将新的点云加入map
    getPointCloudList().insert(str, tempCloud);
    return pointCloud;
}

CPointCloud &PointCloudListMgr::CreateFittingCloud(pcl::PointCloud<pcl::PointXYZRGB> plane)
{
    CPointCloud* pointCloud=new CPointCloud();
    pointCloud->isFittingCloud = true; // 拟合出的点云
    pointCloud->setPointCloud(plane);

    return *pointCloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudListMgr::CloudToPtr(pcl::PointCloud<pcl::PointXYZRGB> cloud)
{
    return pcl::PointCloud<pcl::PointXYZRGB>::Ptr (&cloud);
}
