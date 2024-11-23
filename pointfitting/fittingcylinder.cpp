#include "fittingcylinder.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include<QDebug>

FittingCylinder::FittingCylinder() {
    cylinderCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud_subset.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

    coefficients.reset(new pcl::ModelCoefficients);
    inliers.reset(new pcl::PointIndices);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr FittingCylinder::RANSAC(pcl::PointXYZRGB searchPoint,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudptr){
    //创建KD树用于邻域搜索
    pcl::search::KdTree<pcl::PointXYZRGB> kdtree;
    kdtree.setInputCloud(cloudptr);

    std::vector<int> pointIdxRadiusSearch;//存储邻域点的索引
    std::vector<float> pointRadiusSquaredDistance; // 用于存储找到的点到查询点的平方距离

    // 搜索给定半径内的点
    if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 3) {

        pcl::copyPointCloud(*cloudptr, pointIdxRadiusSearch, *cloud_subset);//在邻域点中实现RANSAC算法

        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kd(new pcl::search::KdTree<pcl::PointXYZRGB>());
        kd->setInputCloud(cloud_subset);
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
        pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
        ne.setInputCloud(cloud_subset);
        ne.setSearchMethod(kd);
        ne.setKSearch(50);
        ne.compute(*normals);

        // 创建RANSAC分割对象
        pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_CYLINDER);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setNormalDistanceWeight (0.1);//设置法向量权重
        seg.setMaxIterations(5000);//设置最大迭代次数
        seg.setDistanceThreshold(distance);//设置阈值
        seg.setInputNormals(normals);
        seg.setInputCloud(cloud_subset);

        // 执行分割
        seg.segment(*inliers, *coefficients);

        //获取圆柱上的所有点云
        for (int i=0;i<cloudptr->size();i++) {
            if(isPointInCylinder(cloudptr->points[i])){
                cylinderCloud->points.push_back(cloudptr->points[i]);
            }
        }
        cylinderCloud->width = cylinderCloud->points.size();
        cylinderCloud->height = 1;
        cylinderCloud->is_dense = true;
        //将圆柱上的点云设置为红色
        for (auto& point : cylinderCloud->points){
            point.r = 255;
            point.g = 0;
            point.b = 0;
        }
    }

    // // 获取圆柱点云的最小和最大边界
    // pcl::PointXYZRGB min_pt, max_pt;
    // pcl::getMinMax3D(*cylinderCloud, min_pt, max_pt);
    // qDebug()<<"222:"<<min_pt.x<<min_pt.y<<min_pt.z;
    // qDebug()<<max_pt.x<<max_pt.y<<max_pt.z;

    // // 计算圆柱的中心点（两个端点的中点）
    // pcl::PointXYZRGB cylinder_center;
    // cylinder_center.x = (min_pt.x + max_pt.x) / 2.0f;
    // cylinder_center.y = (min_pt.y + max_pt.y) / 2.0f;
    // cylinder_center.z = (min_pt.z + max_pt.z) / 2.0f;

    // // 计算圆柱点云的几何中心
    // float cx = 0, cy = 0, cz = 0;
    // int num_points = cloudptr->size();

    // for (const auto& point : cloudptr->points) {
    //     cx += point.x;
    //     cy += point.y;
    //     cz += point.z;
    // }

    // // 计算平均位置（几何中心）
    // cx /= num_points;
    // cy /= num_points;
    // cz /= num_points;

    // 计算圆柱中心：通过起点和终点来计算
    // Eigen::Vector3f cylinder_start(coefficients->values[0] - coefficients->values[3] * radius,
    //                                coefficients->values[1] - coefficients->values[4] * radius,
    //                                coefficients->values[2] - coefficients->values[5] * radius);
    // Eigen::Vector3f cylinder_end(coefficients->values[0] + coefficients->values[3] * radius,
    //                              coefficients->values[1] + coefficients->values[4] * radius,
    //                              coefficients->values[2] + coefficients->values[5] * radius);

    // center = (cylinder_start + cylinder_end) / 2.0f;  // 计算中心点
    // qDebug()<<"11"<<center.x()<<center.y()<<center.z();
    // qDebug()<<coefficients->values[0]<<coefficients->values[1]<<coefficients->values[2];

    // 计算圆柱中心
    //center=Eigen::Vector3f(cx,cy,cz);
    center=Eigen::Vector3f(coefficients->values[0], coefficients->values[1], coefficients->values[2]);

    // 计算圆柱法向量（圆柱轴的方向）
    normal=Eigen::Vector3f(coefficients->values[3], coefficients->values[4], coefficients->values[5]);
    normal.normalize(); // 确保法向量是单位向量

    // 计算圆柱直径
    diameter = 2 * coefficients->values[6];

    // // 计算点云的边界框，以估算高度
    // pcl::PointXYZRGB min_pt, max_pt;
    // pcl::getMinMax3D(*cylinderCloud, min_pt, max_pt);
    // height = pcl::euclideanDistance(min_pt, max_pt);

    // 计算圆柱的高度：通过点云中的投影来获得
    double min_proj = std::numeric_limits<float>::max();
    double max_proj = std::numeric_limits<float>::lowest();

    // 对所有点进行法向量投影，得到最小和最大投影值
    for (size_t i = 0; i < cloud_subset->size(); i++) {
        Eigen::Vector3f point(cloud_subset->points[i].x, cloud_subset->points[i].y, cloud_subset->points[i].z);
        double proj = point.dot(normal); // 计算点到圆柱轴的投影
        min_proj = std::min(min_proj, proj);
        max_proj = std::max(max_proj, proj);
    }

    height = max_proj - min_proj; // 高度是投影值的差

    // // 获取圆柱的起点和终点 (用于计算高度)
    // pcl::PointXYZ p1, p2;
    // p1.x = coefficients->values[0] + 0.5f * radius * normal.x();  // 根据法向量偏移一点，得到圆柱的起点
    // p1.y = coefficients->values[1] + 0.5f * radius * normal.y();
    // p1.z = coefficients->values[2] + 0.5f * radius * normal.z();
    // p2.x = coefficients->values[0] - 0.5f * radius * normal.x();  // 终点
    // p2.y = coefficients->values[1] - 0.5f * radius * normal.y();
    // p2.z = coefficients->values[2] - 0.5f * radius * normal.z();

    // // 计算圆柱的高度
    // height = pcl::euclideanDistance(p1, p2);

    return cylinderCloud;
}

bool FittingCylinder::isPointInCylinder(const pcl::PointXYZRGB& point){
    // 提取圆柱参数
    // Eigen::Vector3f axis_point(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
    // Eigen::Vector3f axis_direction(coefficients->values[3], coefficients->values[4], coefficients->values[5]);
    double r = coefficients->values[6];//圆柱半径

    // 将目标点转为 Eigen 向量
    Eigen::Vector3f p(point.x, point.y, point.z);

    // 计算从轴点到目标点的向量
    Eigen::Vector3f v = p - center;

    // 计算点到轴线的距离
    Eigen::Vector3f cross_product = v.cross(normal);//v和normal的叉积
    double distance = cross_product.norm() / normal.norm();  // 点到轴线的垂直距离

    // 判断点是否在圆柱表面
    return std::abs(distance - r) <= radius;
}

void FittingCylinder::setRadius(double rad){
    radius=rad;
}

void FittingCylinder::setDistance(double dis){
    distance=dis;
}

Eigen::Vector3f FittingCylinder::getNormal(){
    return normal;
}

Eigen::Vector3f FittingCylinder::getCenter(){
    return center;
}

double FittingCylinder::getDiameter(){
    return diameter;
}

double FittingCylinder::getHeight(){
    return height;
}
