#ifndef FILEMGR_H
#define FILEMGR_H

#include <qmap>
#include <qvector>
#include <pcl/point_cloud.h>     // PCL 的点云类
#include <pcl/point_types.h>     // PCL 的点类型定义
#include <pcl/io/pcd_io.h>       // PCL 的 PCD 文件输入输出类
#include <pcl/visualization/pcl_visualizer.h> // PCL 的可视化工具

class FileMgr
{
public:
    FileMgr();
    QMap<QString, bool>& getModelFileMap();
    QMap<QString, bool>& getMeasuredFileMap();
    QMap<QString, bool>& getContentItemMap();
    QMap<QString, bool>& getIdentifyItemMap();
    pcl::PointCloud<pcl::PointXYZ>::Ptr& getCloudPtr();
    pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudptr;
private:
    QMap<QString, bool> modelFileLMap;
    QMap<QString, bool> measuredFileMap;
    QMap<QString, bool> contentItemMap;
    QMap<QString, bool> identifyItemMap;

};

#endif // FILEMGR_H
