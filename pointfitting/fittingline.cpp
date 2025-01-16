#include "fittingline.h"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/centroid.h>
#include <pcl/surface/convex_hull.h>
#include <Eigen/Dense>
#include <pcl/common/pca.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/random.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/common.h>

#include <QMessageBox>

FittingLine::FittingLine() {
    //创建点云智能指针
    lineCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud_subset.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

    coefficients.reset(new pcl::ModelCoefficients);
    inliers.reset(new pcl::PointIndices);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr FittingLine::RANSAC(pcl::PointXYZRGB searchPoint,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudptr){
    //创建KD树用于邻域搜索
    pcl::search::KdTree<pcl::PointXYZRGB> kdtree;
    kdtree.setInputCloud(cloudptr);

    std::vector<int> pointIdxRadiusSearch;//存储邻域点的索引
    std::vector<float> pointRadiusSquaredDistance; // 用于存储找到的点到查询点的平方距离

    // 搜索给定半径内的点
    if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 1) {

        pcl::copyPointCloud(*cloudptr, pointIdxRadiusSearch, *cloud_subset);//在邻域点中实现RANSAC算法
        // 创建RANSAC分割对象
        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_LINE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(5000);//设置最大迭代次数
        seg.setDistanceThreshold(distance);//设置阈值
        // 提供输入点云
        seg.setInputCloud(cloud_subset);

        // 执行分割
        seg.segment(*inliers, *coefficients);

        normal=Eigen::Vector3f(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
        normal.normalize();
        onePoint=Eigen::Vector3f(coefficients->values[3], coefficients->values[4], coefficients->values[5]);

        //获取平面上的所有点云
        for (int i=0;i<cloudptr->size();i++) {
            if(isPointInLine(cloudptr->points[i])){
                lineCloud->points.push_back(cloudptr->points[i]);
            }
        }
        lineCloud->width = lineCloud->points.size();
        lineCloud->height = 1;
        lineCloud->is_dense = true;
        //将平面上的点云设置为红色
        for (auto& point : lineCloud->points){
            point.r = 255;
            point.g = 0;
            point.b = 0;
        }

        // qDebug()<<"00:"<<beginPoint.x()<<beginPoint.y()<<beginPoint.z();
        // qDebug()<<"00:"<<endPoint.x()<<endPoint.y()<<endPoint.z();

        // qDebug()<<"00:"<<beginPoint.x()<<beginPoint.y()<<beginPoint.z();
        // qDebug()<<"00:"<<endPoint.x()<<endPoint.y()<<endPoint.z();

        // 获取点云的最小和最大边界
        pcl::PointXYZRGB min_pt, max_pt;
        pcl::getMinMax3D(*lineCloud, min_pt, max_pt);

        Eigen::Vector3f min_point(min_pt.x, min_pt.y, min_pt.z);
        Eigen::Vector3f max_point(max_pt.x, max_pt.y, max_pt.z);

        // 计算直线与边界的交点
        // 直线方程：P(t) = P0 + t * V，其中P0是一个点，V是方向向量
        // 求与边界的交点，计算t_min和t_max

        // 计算各个方向上的t值
        float t_min_x = (min_point.x() - onePoint.x()) / normal.x();
        float t_max_x = (max_point.x() - onePoint.x()) / normal.x();

        float t_min_y = (min_point.y() - onePoint.y()) / normal.y();
        float t_max_y = (max_point.y() - onePoint.y()) / normal.y();

        float t_min_z = (min_point.z() - onePoint.z()) / normal.z();
        float t_max_z = (max_point.z() - onePoint.z()) / normal.z();

        // 获取每个轴上的最小和最大t
        float t_min = std::min({t_min_x, t_min_y, t_min_z});
        float t_max = std::max({t_max_x, t_max_y, t_max_z});

        // 计算起点和终点
        beginPoint = onePoint + t_min * normal;
        endPoint = onePoint + t_max * normal;

        // 打印调试信息
        qDebug() << "Begin Point: " << beginPoint.x() << ", " << beginPoint.y() << ", " << beginPoint.z();
        qDebug() << "End Point: " << endPoint.x() << ", " << endPoint.y() << ", " << endPoint.z();

        return lineCloud;

    }else{
        QMessageBox *messagebox=new QMessageBox();
        messagebox->setText("输入的邻域或距离阈值太小，\n请重新输入！");
        messagebox->setIcon(QMessageBox::Warning);
        messagebox->show();
        messagebox->exec();
        PCL_ERROR("Couldn't find more points within radius\n");
        return nullptr;
    }
}

bool FittingLine::FittingLine::isPointInLine(const pcl::PointXYZRGB& point){
    Eigen::Vector3f pointVec(point.x, point.y, point.z); // 点的坐标
    Eigen::Vector3f pointToLine = pointVec - onePoint; // 点到直线的向量
    double d=pointToLine.cross(normal).norm() / normal.norm(); // 计算点到直线的距离
    return fabs(d) <= distance;
}

void FittingLine::setRadius(double rad){
    radius=rad;
}

void FittingLine::setDistance(double dis){
    distance=dis;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr FittingLine::getLineCloud(){
    return lineCloud;
}

Eigen::Vector3f FittingLine::getBegin(){
    return beginPoint;
}

Eigen::Vector3f FittingLine::getEnd(){
    return endPoint;
}
