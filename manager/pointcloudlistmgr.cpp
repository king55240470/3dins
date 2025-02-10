#include "pointcloudlistmgr.h"

PointCloudListMgr::PointCloudListMgr() {}

QMap<QString, CPointCloud*> &PointCloudListMgr::getFileCloudMap()
{
    return fileCloudMap;
}
CPointCloud* PointCloudListMgr::CreateCloudFromFile(QString str)
{
    QString suffix = str.section('.', -1, -1); // 从最后一个'.'到最后一个字符，获取后缀名
    QString fileName = str.section('/', -1, -1);
    // 创建并加载RGB点云
    if(suffix == "pcd")
        pcl::io::loadPCDFile(str.toStdString(), tempCloud);
    else if(suffix == "ply")
        pcl::io::loadPLYFile(str.toStdString(), tempCloud);

    // 创建新的点云实体
    CPointCloud* CloudEntity=new CPointCloud();
    CloudEntity->isFileCloud = true;
    CloudEntity->setPointCloud(tempCloud);
    CloudEntity->m_strAutoName = fileName;

    // 将新的点云实体加入FileCloudMap，将rgb点云加入rgbCloudMap
    getFileCloudMap().insert(str, CloudEntity);
    return CloudEntity;
}

void PointCloudListMgr::DeleteFileCloud(QString filepath)
{
    getFileCloudMap().remove(filepath); // 当文件窗口删除文件时，这里对应删除

}

CPointCloud* PointCloudListMgr::CreateFittingCloud(pcl::PointCloud<pcl::PointXYZRGB> plane)
{
    CPointCloud* CloudEntity=new CPointCloud();
    CloudEntity->setPointCloud(plane);

    return CloudEntity;
}

CPointCloud *PointCloudListMgr::CreateCompareCloud(pcl::PointCloud<pcl::PointXYZRGB> cloud)
{
    CPointCloud* CloudEntity=new CPointCloud();
    CloudEntity->isComparsionCloud = true; // 对比生成的点云
    CloudEntity->setPointCloud(cloud);
    CloudEntity->m_strAutoName += "(对比)";
    return CloudEntity;
}


CPointCloud *PointCloudListMgr::CreateAlignCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
    CPointCloud* CloudEntity = new CPointCloud();
    CloudEntity->isAlignCloud = true;
    CloudEntity->setPointCloud(cloud);  // 直接存储 shared_ptr，避免拷贝
    return CloudEntity;
}
