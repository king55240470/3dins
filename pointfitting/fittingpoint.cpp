#include "fittingpoint.h"

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>

#include <QMessageBox>

FittingPoint::FittingPoint() {}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr FittingPoint::RANSAC(pcl::PointXYZRGB searchPoint,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudptr){
    // 创建 K-D 树
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    kdtree.setInputCloud(cloudptr);

    // 准备搜索参数
    int K = 1; // 找最近的 1 个点
    std::vector<int> indices(K);      // 最近点的索引
    std::vector<float> distances(K);  // 最近点的距离

    // 搜索最近邻
    if (kdtree.nearestKSearch(searchPoint, K, indices, distances) > 0) {
        point=QVector4D(cloudptr->points[indices[0]].x,cloudptr->points[indices[0]].y,cloudptr->points[indices[0]].z,0);
        cloudptr->points[indices[0]].r=1;
        cloudptr->points[indices[0]].g=0;
        cloudptr->points[indices[0]].b=0;
    }else{
        QMessageBox *messagebox=new QMessageBox();
        messagebox->setText("未找到最近点！");
        messagebox->setIcon(QMessageBox::Warning);
        messagebox->show();
        messagebox->exec();
        return nullptr;
    }
    return cloudptr;
}
