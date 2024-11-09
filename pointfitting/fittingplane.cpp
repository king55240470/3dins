#include "pointfitting/fittingplane.h"

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

#include <QMessageBox>

FittingPlane::FittingPlane()
{
    //创建点云智能指针
    planeCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud_subset.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

    coefficients.reset(new pcl::ModelCoefficients);
    inliers.reset(new pcl::PointIndices);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr FittingPlane::RANSAC(pcl::PointXYZRGB searchPoint,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudptr)
{
    //创建KD树用于邻域搜索
    pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    kdtree.setInputCloud(cloudptr);

    std::vector<int> pointIdxRadiusSearch;//存储邻域点的索引
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
qDebug()<<"here is ok 2";
        // 提供输入点云
        seg.setInputCloud(cloud_subset);

        // 执行分割
        seg.segment(*inliers, *coefficients);

        for (int i=0;i<cloudptr->size();i++) {
            if(isPointInPlane(cloudptr->points[i])){
                planeCloud->points.push_back(cloudptr->points[i]);
            }
        }
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

        return planeCloud;
    } else {
        QMessageBox *messagebox=new QMessageBox();
        messagebox->setText("输入的邻域或距离阈值太小，\n请重新输入！");
        messagebox->setIcon(QMessageBox::Warning);
        messagebox->show();
        messagebox->exec();
        PCL_ERROR("Couldn't find more points within radius\n");
        return 0;
    }
}

bool FittingPlane::isPointInPlane(const pcl::PointXYZRGB& point){
    double d = coefficients->values[0] * point.x +
                      coefficients->values[1] * point.y +
                      coefficients->values[2] * point.z +
                      coefficients->values[3];
    return std::abs(d) <= 0.01;
}

void FittingPlane::setRadious(double rad){
    radious=rad;
}

void FittingPlane::setDistance(double dis){
    distance=dis;
}
