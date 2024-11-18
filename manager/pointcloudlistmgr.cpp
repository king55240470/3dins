#include "pointcloudlistmgr.h"

PointCloudListMgr::PointCloudListMgr() {}

QMap<QString, CPointCloud*> &PointCloudListMgr::getFileCloudMap()
{
    return fileCloudMap;
}
CPointCloud* PointCloudListMgr::CreateCloudFromFile(QString str)
{
    QString suffix = str.section('.', -1, -1); // 从最后一个'.'到最后一个字符，获取后缀名

    // 创建并加载RGB点云
    if(suffix == "pcd")
        pcl::io::loadPCDFile(str.toStdString(), tempCloud);
    else if(suffix == "ply")
        pcl::io::loadPLYFile(str.toStdString(), tempCloud);

    // 创建新的点云实体
    CPointCloud* CloudEntity=new CPointCloud();
    CloudEntity->isFileCloud = true;
    CloudEntity->setPointCloud(tempCloud);

    // 将新的点云实体加入map
    getFileCloudMap().insert(str, CloudEntity);
    return CloudEntity;
}

void PointCloudListMgr::DeleteFileCloud(QString filepath)
{
    for(auto item = getFileCloudMap().begin();item != getFileCloudMap().end();item++){
        if(filepath == item.key()){
            getFileCloudMap().erase(item); // 当文件窗口删除文件时，这里对应删除
        }
    }
}

CPointCloud* PointCloudListMgr::CreateFittingCloud(pcl::PointCloud<pcl::PointXYZRGB> plane)
{
    CPointCloud* CloudEntity=new CPointCloud();
    CloudEntity->isFittingCloud = true; // 拟合出的点云
    CloudEntity->setPointCloud(plane);

    return CloudEntity;
}

CPointCloud *PointCloudListMgr::CreateComparsionCloud(pcl::PointCloud<pcl::PointXYZRGB> cloud)
{
    CPointCloud* CloudEntity=new CPointCloud();
    CloudEntity->isFittingCloud = true; // 对比生成的点云
    CloudEntity->setPointCloud(cloud);
    return CloudEntity;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudListMgr::CloudToPtr(pcl::PointCloud<pcl::PointXYZRGB> cloud)
{
    return pcl::PointCloud<pcl::PointXYZRGB>::Ptr (&cloud);
}
