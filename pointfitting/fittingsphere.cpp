#include "fittingsphere.h"

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

FittingSphere::FittingSphere() {
    sphereCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud_subset.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

    coefficients.reset(new pcl::ModelCoefficients);
    inliers.reset(new pcl::PointIndices);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr FittingSphere::RANSAC(pcl::PointXYZRGB searchPoint,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudptr){

    //清除数据
    cloud_subset->clear();
    sphereCloud->clear();

    //创建KD树用于邻域搜索
    pcl::search::KdTree<pcl::PointXYZRGB> kdtree;
    kdtree.setInputCloud(cloudptr);

    std::vector<int> pointIdxRadiusSearch;//存储邻域点的索引
    std::vector<float> pointRadiusSquaredDistance; // 用于存储找到的点到查询点的平方距离

    // 搜索给定半径内的点
    if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 3) {
        pcl::copyPointCloud(*cloudptr, pointIdxRadiusSearch, *cloud_subset);//在邻域点中实现RANSAC算法
        // 创建RANSAC分割对象
        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_SPHERE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(5000);//设置最大迭代次数
        seg.setDistanceThreshold(distance);//设置阈值
        // 提供输入点云
        seg.setInputCloud(cloud_subset);

        // 执行分割
        seg.segment(*inliers, *coefficients);

        //获取球上的所有点云
        for (int i=0;i<cloudptr->size();i++) {
            if(isPointInSphere(cloudptr->points[i])){
                sphereCloud->points.push_back(cloudptr->points[i]);
            }
        }
        sphereCloud->width = sphereCloud->points.size();
        sphereCloud->height = 1;
        sphereCloud->is_dense = true;
        //将球上的点云设置为红色
        for (auto& point : sphereCloud->points){
            point.r = 255;
            point.g = 0;
            point.b = 0;
        }

        //计算球中心
        center=Eigen::Vector3f(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
        //计算球半径
        r=coefficients->values[3];

        return sphereCloud;

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

bool FittingSphere::isPointInSphere(const pcl::PointXYZRGB& point){
    double d=std::sqrt(std::pow(point.x - center.x(), 2) +
                       std::pow(point.y - center.y(), 2) +
                       std::pow(point.z - center.z(), 2));

    return fabs(d - radius) < 0.01;
}

void FittingSphere::setRadius(double rad){
    radius=rad;
}

void FittingSphere::setDistance(double dis){
    distance=dis;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr FittingSphere::getSphereCloud(){
    return sphereCloud;
}

Eigen::Vector3f FittingSphere::getCenter(){
    return center;
}

double FittingSphere::getRad(){
    return r;
}
