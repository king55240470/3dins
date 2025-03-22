#include "fittingcircle.h"

#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>

#include <Eigen/Dense>
#include <QMessageBox>

FittingCircle::FittingCircle() {
    circleCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud_subset.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

    coefficients.reset(new pcl::ModelCoefficients);
    inliers.reset(new pcl::PointIndices);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr FittingCircle::RANSAC(pcl::PointXYZRGB searchPoint,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudptr){

    //清除数据
    cloud_subset->clear();
    circleCloud->clear();

    //创建KD树用于邻域搜索
    pcl::search::KdTree<pcl::PointXYZRGB> kdtree;
    kdtree.setInputCloud(cloudptr);

    std::vector<int> pointIdxRadiusSearch;//存储邻域点的索引
    std::vector<float> pointRadiusSquaredDistance; // 用于存储找到的点到查询点的平方距离

    // 搜索给定半径内的点
    if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 3) {

        pcl::copyPointCloud(*cloudptr, pointIdxRadiusSearch, *cloud_subset);

        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_CIRCLE3D);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(5000);
        seg.setDistanceThreshold(distance);
        seg.setInputCloud(cloud_subset);

        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        seg.segment(*inliers, *coefficients);

        //计算圆中心
        center=Eigen::Vector3f(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
        //计算圆半径
        r=coefficients->values[3];
        //计算圆法向量
        normal=Eigen::Vector3f(coefficients->values[4], coefficients->values[5], coefficients->values[6]);

        //获取圆上的所有点云
        for (int i=0;i<cloudptr->size();i++) {
            if(isPointInCircle(cloudptr->points[i])){
                circleCloud->points.push_back(cloudptr->points[i]);
            }
        }
        circleCloud->width = circleCloud->points.size();
        circleCloud->height = 1;
        circleCloud->is_dense = true;
        //将球上的点云设置为红色
        for (auto& point : circleCloud->points){
            point.r = 255;
            point.g = 0;
            point.b = 0;
        }

        return circleCloud;

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

bool FittingCircle::isPointInCircle(const pcl::PointXYZRGB& point){
    double d=std::sqrt(std::pow(point.x - center.x(), 2) +
                       std::pow(point.y - center.y(), 2) +
                       std::pow(point.z - center.z(), 2));

    return fabs(d - r) < 0.01;
}

void FittingCircle::setRadius(double rad){
    radius=rad;
}

void FittingCircle::setDistance(double dis){
    distance=dis;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr FittingCircle::getCircleCloud(){
    return circleCloud;
}

Eigen::Vector3f FittingCircle::getCenter(){
    return center;
}

double FittingCircle::getRad(){
    return r;
}

Eigen::Vector3f FittingCircle::getNormal(){
    return normal;
}
