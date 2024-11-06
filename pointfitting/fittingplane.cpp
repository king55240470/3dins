#include "pointfitting/fittingplane.h"

#include<QWindow>
#include<QVBoxLayout>

#include<pcl/io/pcd_io.h>// PCL 的 PCD 文件输入输出类
#include<pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <boost/make_shared.hpp>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/sample_consensus/ransac.h>

FittingPlane::FittingPlane()
{
    //创建点云智能指针
    cloudptr.reset(new PointCloudT);
    planeCloud.reset(new PointCloudT);
    cloud_subset.reset(new PointCloudT);

    //从指定路径加载 PCD 文件到点云对象中
    pcl::io::loadPLYFile("E:\\pcl\\box.ply", *cloudptr);
    //pcl::io::loadPCDFile("D:/1_university/Code/Qt code/maize.pcd", *cloudptr);

    coefficients.reset(new pcl::ModelCoefficients);
    inliers.reset(new pcl::PointIndices);

    RANSAC();

    // 打印平面方程的系数
    std::cout << "Plane coefficients: " << coefficients->values[0] << " "
              << coefficients->values[1] << " " << coefficients->values[2] << " "
              << coefficients->values[3] << std::endl;

    visualizePlane();
}

void FittingPlane::RANSAC()
{
    //创建KD树用于邻域搜索
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloudptr);

    // 初始化一个点来搜索其邻域（这里选择点云中的第一个点作为示例）
    searchPoint = cloudptr->points[500000];
    std::vector<int> pointIdxRadiusSearch;
    double radius = 0.1f; // 设置搜索半径
    std::vector<float> pointRadiusSquaredDistance; // 用于存储找到的点到查询点的平方距离

    // 搜索给定半径内的点
    if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {

        pcl::copyPointCloud(*cloudptr, pointIdxRadiusSearch, *cloud_subset);//在邻域点中实现RANSAC算法

        // 创建RANSAC分割对象
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(1000);//设置最大迭代次数
        seg.setDistanceThreshold(0.001);//设置阈值

        // 提供输入点云
        seg.setInputCloud(cloud_subset);

        // 执行分割
        seg.segment(*inliers, *coefficients);

        for (int i=0;i<cloudptr->size();i++) {
            if(isPointInPlane(cloudptr->points[i])){
                planeCloud->points.push_back(cloudptr->points[i]);
            }
        }
        // for (int index : inliers->indices) {
        //     planeCloud->points.push_back(cloudptr->points[index]);
        // }
        planeCloud->width = planeCloud->points.size();
        planeCloud->height = 1;
        planeCloud->is_dense = true;
    } else {
        PCL_ERROR("Couldn't find more points within radius\n");
        return;
    }
}

bool FittingPlane::isPointInPlane(const PointT& point){
    double distance = coefficients->values[0] * point.x +
                      coefficients->values[1] * point.y +
                      coefficients->values[2] * point.z +
                      coefficients->values[3];
    return std::abs(distance) <= 0.01;
}

void FittingPlane::visualizePlane() {

    pcl::visualization::PCLVisualizer viewer("3D Viewer");

    viewer.addPointCloud<pcl::PointXYZ>(cloudptr, "allCloud");
    viewer.addPointCloud<pcl::PointXYZ>(planeCloud, "planeCloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "planeCloud"); // 设置拟合平面内点的颜色为红色
    viewer.setBackgroundColor(0.0, 0.0, 0.0); // 设置背景色为黑色

    while (!viewer.wasStopped()) {
        viewer.spinOnce(100);
    }
}
