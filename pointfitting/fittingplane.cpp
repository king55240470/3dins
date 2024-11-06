#include "pointfitting/fittingplane.h"
#include "pointfitting/setdatawidget.h"

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
    // cloudptr.reset(new pcl::PointCloud<pcl::PointXYZ>);
    // planeCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    // cloud_subset.reset(new pcl::PointCloud<pcl::PointXYZ>);

    // //从指定路径加载 PCD 文件到点云对象中
    // pcl::io::loadPLYFile("D:/1_university/Code/Qt code/02G1_actual_cloud.ply", *cloudptr);
    // //pcl::io::loadPCDFile("D:/1_university/Code/Qt code/maize.pcd", *cloudptr);
    //从指定路径加载 PCD 文件到点云对象中
    //pcl::io::loadPLYFile("E:\\pcl\\box.ply", *cloudptr);
    //pcl::io::loadPCDFile("D:/1_university/Code/Qt code/maize.pcd", *cloudptr);

    coefficients.reset(new pcl::ModelCoefficients);
    inliers.reset(new pcl::PointIndices);

    p_setDataWidget=new setDataWidget();

    //RANSAC();

}

void FittingPlane::RANSAC(pcl::PointXYZRGB searchPoint,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudptr)
{
    p_setDataWidget->setPlaneData();

    //创建KD树用于邻域搜索
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    kdtree.setInputCloud(cloudptr);

    // 初始化一个点来搜索其邻域（这里选择点云中的第一个点作为示例）
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance; // 用于存储找到的点到查询点的平方距离

    // 搜索给定半径内的点
    if (kdtree.radiusSearch(searchPoint, radious, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 3) {

        pcl::copyPointCloud(*cloudptr, pointIdxRadiusSearch, *cloud_subset);//在邻域点中实现RANSAC算法

        // 创建RANSAC分割对象
        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(1000);//设置最大迭代次数
        seg.setDistanceThreshold(distance);//设置阈值

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

        for (auto& point : planeCloud->points){
            point.r = 255;
            point.g = 0;
            point.b = 0;
        }

        // 打印平面方程的系数
        std::cout << "Plane coefficients: " << coefficients->values[0] << " "
                  << coefficients->values[1] << " " << coefficients->values[2] << " "
                  << coefficients->values[3] << std::endl;

    } else {
        PCL_ERROR("Couldn't find more points within radius\n");
        return;
    }
}

bool FittingPlane::isPointInPlane(const pcl::PointXYZRGB& point){
    double d = coefficients->values[0] * point.x +
                      coefficients->values[1] * point.y +
                      coefficients->values[2] * point.z +
                      coefficients->values[3];
    return std::abs(d) <= 0.01;
}

double &FittingPlane::getRadious(){
    return radious;
}

int &FittingPlane::getDistance(){
    return distance;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr FittingPlane::getPlaneCloud(){
    return planeCloud;
}
