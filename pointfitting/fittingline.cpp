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
    if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 3) {

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

        // //计算起点和终点
        // double minDistance = std::numeric_limits<float>::max();
        // double maxDistance = std::numeric_limits<float>::lowest();
        // pcl::PointXYZRGB closestPoint, farthestPoint;

        // for (const auto& point : lineCloud->points) {
        //     Eigen::Vector3f pointVec(point.x, point.y, point.z);

        //     // 计算点到直线的向量
        //     Eigen::Vector3f pointToLine = pointVec - onePoint;
        //     // 计算投影比例
        //     float t = pointToLine.dot(normal) / normal.squaredNorm();
        //     // 计算投影点
        //     Eigen::Vector3f projection = onePoint + t * normal;

        //     double d=(projection - pointVec).norm();

        //     // 更新最小和最大距离
        //     if (d < minDistance) {
        //         minDistance = d;
        //         closestPoint = point;
        //     }
        //     if (d > maxDistance) {
        //         maxDistance = d;
        //         farthestPoint = point;
        //     }
        // }

        // 获取点云的最小和最大边界
        Eigen::Vector3f min_point, max_point;
        getCloudBounds(lineCloud, min_point, max_point);

        // 获取直线的起点和终点
        qDebug()<<"00:"<<beginPoint.x()<<beginPoint.y()<<beginPoint.z();
        qDebug()<<"00:"<<endPoint.x()<<endPoint.y()<<endPoint.z();
        getLineEndpoints(onePoint, normal, min_point, max_point, beginPoint, endPoint);
        qDebug()<<"00:"<<beginPoint.x()<<beginPoint.y()<<beginPoint.z();
        qDebug()<<"00:"<<beginPoint.x()<<beginPoint.y()<<beginPoint.z();

        // 返回距离最小的点作为起点，最大距离的点作为终点
        // beginPoint = Eigen::Vector3f(closestPoint.x,closestPoint.y,closestPoint.z);
        // endPoint = Eigen::Vector3f(farthestPoint.x,farthestPoint.y,farthestPoint.z);

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

// 获取点云的边界（最小点和最大点）
void FittingLine::getCloudBounds(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, Eigen::Vector3f& min_point, Eigen::Vector3f& max_point) {
    pcl::PointXYZRGB min_pt, max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);  // 获取点云的最小和最大点
    min_point = Eigen::Vector3f(min_pt.x, min_pt.y, min_pt.z);
    max_point = Eigen::Vector3f(max_pt.x, max_pt.y, max_pt.z);
}

// 获取直线的起点和终点
void FittingLine::getLineEndpoints(const Eigen::Vector3f& linePoint, const Eigen::Vector3f& direction, const Eigen::Vector3f& min_point, const Eigen::Vector3f& max_point, Eigen::Vector3f& startPoint, Eigen::Vector3f& endPoint) {
        // 计算x轴的交点
        float t_min_x = (min_point.x() - linePoint.x()) / direction.x();
        float t_max_x = (max_point.x() - linePoint.x()) / direction.x();

        // 计算y轴的交点
        float t_min_y = (min_point.y() - linePoint.y()) / direction.y();
        float t_max_y = (max_point.y() - linePoint.y()) / direction.y();

        // 计算z轴的交点
        float t_min_z = (min_point.z() - linePoint.z()) / direction.z();
        float t_max_z = (max_point.z() - linePoint.z()) / direction.z();

        // 获取每个轴的最小最大 t
        float t_min = std::min({t_min_x, t_min_y, t_min_z});
        float t_max = std::max({t_max_x, t_max_y, t_max_z});

        // 计算起点和终点
        startPoint = linePoint + direction * t_min;
        endPoint = linePoint + direction * t_max;
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
