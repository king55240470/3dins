#ifndef POINTCLOUDLISTMGR_H
#define POINTCLOUDLISTMGR_H

#include "geometry/centitytypes.h"
#include "pointfitting/fittingplane.h"

#include <QVector>
#include <QMap>
#include <pcl/common/common.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/distances.h>  // PCL距离计算函数

class PointCloudListMgr
{
public:
    PointCloudListMgr();
    QMap<QString, CPointCloud*> &getFileCloudMap();

    CPointCloud* CreateCloudFromFile(QString str); // 给打开的文件分配RGB点云，并生成点云对象
    void DeleteFileCloud(QString filepath); // 删除文件对应的点云
    CPointCloud* CreateFittingCloud(pcl::PointCloud<pcl::PointXYZRGB> plane);
    CPointCloud* CreateCompareCloud(pcl::PointCloud<pcl::PointXYZRGB> cloud);
    CPointCloud *CreateAlignCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);
    CPointCloud *CreateReconstructedCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);
    CPointCloud* CreateFilterCloud(pcl::PointCloud<pcl::PointXYZRGB>& cloud);
    CPointCloud* CreateCompleteCloud(pcl::PointCloud<pcl::PointXYZRGB>& cloud);

    pcl::PointCloud<pcl::PointXYZRGB>& getTempCloud(){
        return tempCloud;
    };

private:
    QMap<QString, CPointCloud*> fileCloudMap; // 将打开的文件和点云实体绑定

    pcl::PointCloud<pcl::PointXYZRGB> tempCloud; // 临时点云对象
};

#endif // POINTCLOUDLISTMGR_H
