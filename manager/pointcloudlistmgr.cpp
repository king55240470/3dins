#include "pointcloudlistmgr.h"
#include<QDir>
#include<QThread>
PointCloudListMgr::PointCloudListMgr() {}

QMap<QString, CPointCloud*> &PointCloudListMgr::getFileCloudMap()
{
    return fileCloudMap;
}
/*
CPointCloud* PointCloudListMgr::CreateCloudFromFile(QString str)
{
    QString suffix = str.section('.', -1, -1); // 从最后一个'.'到最后一个字符，获取后缀名
    QString fileName = str.section('/', -1, -1);

    // 创建并加载RGB点云
        if(suffix == "pcd")
            pcl::io::loadPCDFile(QDir::toNativeSeparators(str).toStdString(), tempCloud);
        else if(suffix == "ply")
            pcl::io::loadPLYFile(QDir::toNativeSeparators(str).toStdString(), tempCloud);


    qDebug()<<"路径"<<str;
    qDebug()<<"后缀"<<suffix;
    qDebug()<<"文件名"<<fileName;
    qDebug()<<"临时点云点数"<<tempCloud.size();
    // 创建新的点云实体
    CPointCloud* CloudEntity=new CPointCloud();
    CloudEntity->isFileCloud = true;
    CloudEntity->setPointCloud(tempCloud);
    CloudEntity->m_strAutoName = fileName;

    // 将新的点云实体加入FileCloudMap，将rgb点云加入rgbCloudMap
    getFileCloudMap().insert(str, CloudEntity);
    return CloudEntity;
}
*/
CPointCloud* PointCloudListMgr::CreateCloudFromFile(QString str)
{
    QString suffix = str.section('.', -1, -1);
    QString fileName = str.section('/', -1, -1);
    pcl::PointCloud<pcl::PointXYZRGB> tempCloud;
    int retryCount = 0;
    const int maxRetries = 10;       // 最大重试次数
    const int retryInterval = 200;   // 重试间隔(毫秒)
    bool readSuccess = false;

    // 检查文件是否存在并可读
    QFileInfo fileInfo(str);
    if(!fileInfo.exists()) {
        qDebug() << "File does not exist:" << str;
        return nullptr;
    }

    // 重试机制，等待文件完全写入
    while(retryCount < maxRetries && !readSuccess) {
        // 检查文件大小是否稳定
        qint64 prevSize = fileInfo.size();
        QThread::msleep(retryInterval);
        fileInfo.refresh();

        if(prevSize > 0 && prevSize == fileInfo.size()) {
            // 文件大小稳定，尝试读取
            try {
                if(suffix == "pcd") {
                    if(pcl::io::loadPCDFile(QDir::toNativeSeparators(str).toStdString(), tempCloud) == 0) {
                        readSuccess = true;
                    }
                }
                else if(suffix == "ply") {
                    if(pcl::io::loadPLYFile(QDir::toNativeSeparators(str).toStdString(), tempCloud) == 0) {
                        readSuccess = true;
                    }
                }

                if(readSuccess && tempCloud.empty()) {
                    qDebug() << "Warning: File read successfully but cloud is empty:" << str;
                    readSuccess = false; // 视为失败，继续重试
                }
            }
            catch(const std::exception& e) {
                qDebug() << "Exception while reading file:" << e.what();
            }
        }

        if(!readSuccess) {
            retryCount++;
            qDebug() << "Retry reading" << str << "attempt" << retryCount;
            QThread::msleep(retryInterval);
            fileInfo.refresh(); // 刷新文件信息
        }
    }

    if(!readSuccess) {
        qDebug() << "Failed to read file after" << maxRetries << "attempts:" << str;
        return nullptr;
    }

    qDebug() << "路径:" << str;
    qDebug() << "后缀:" << suffix;
    qDebug() << "文件名:" << fileName;
    qDebug() << "临时点云点数:" << tempCloud.size();

    // 创建新的点云实体
    CPointCloud* CloudEntity = new CPointCloud();
    CloudEntity->isFileCloud = true;
    CloudEntity->setPointCloud(tempCloud);
    CloudEntity->m_strAutoName = fileName;

    // 将新的点云实体加入FileCloudMap
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
    CloudEntity->m_EntityType=enPointCloud;
    return CloudEntity;
}


CPointCloud *PointCloudListMgr::CreateAlignCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
    CPointCloud* CloudEntity = new CPointCloud();
    CloudEntity->isAlignCloud = true;
    CloudEntity->setPointCloud(cloud);  // 直接存储 shared_ptr，避免拷贝
    CloudEntity->m_strAutoName += "(对齐)";
    CloudEntity->m_EntityType=enPointCloud;
    return CloudEntity;
}

CPointCloud *PointCloudListMgr::CreateReconstructedCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
{
    CPointCloud* CloudEntity = new CPointCloud();
    CloudEntity->isReconstructedCloud = true;
    CloudEntity->setPointCloud(cloud);  // 直接存储 shared_ptr，避免拷贝
    CloudEntity->m_strAutoName += "(重建)";
    return CloudEntity;
}

CPointCloud *PointCloudListMgr::CreateFilterCloud(pcl::PointCloud<pcl::PointXYZRGB> &cloud)
{
    CPointCloud* CloudEntity = new CPointCloud();
    CloudEntity->isFilteredCloud = true;
    CloudEntity->setPointCloud(cloud);
    return CloudEntity;
}

CPointCloud *PointCloudListMgr::CreateCompleteCloud(pcl::PointCloud<pcl::PointXYZRGB>& cloud)
{
    CPointCloud* CloudEntity = new CPointCloud();
    CloudEntity->isCompleteCloud = true;
    CloudEntity->setPointCloud(cloud);
    CloudEntity->m_strAutoName += "(补齐)";
    return CloudEntity;
}
