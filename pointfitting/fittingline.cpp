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

        // 计算点云的中心点
        pcl::PCA<pcl::PointXYZRGB> pca;
        pca.setInputCloud(cloud_subset);

        // 获取PCA计算得到的主方向（直线方向）
        //normal = pca.getEigenVectors().col(0);
        normal=Eigen::Vector3f(coefficients->values[3], coefficients->values[4], coefficients->values[5]);
        normal.normalize();
        onePoint=Eigen::Vector3f(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
        //onePoint=Eigen::Vector3f(searchPoint.x,searchPoint.y,searchPoint.z);

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

        // 初始化最大最小投影值
        float max_projection = -std::numeric_limits<float>::infinity();
        float min_projection = std::numeric_limits<float>::infinity();

        // 初始化最大最小投影点
        pcl::PointXYZRGB max_point, min_point;

        // 遍历点云中的每个点
        for (const auto& point : lineCloud->points) {
            // 计算当前点的投影值
            float projection = (point.getVector3fMap() - onePoint).dot(normal);

            // 更新最大最小投影值和对应点
            if (projection > max_projection) {
                max_projection = projection;
                max_point = point;  // 记录投影值最大的点
            }
            if (projection < min_projection) {
                min_projection = projection;
                min_point = point;  // 记录投影值最小的点
            }
        }

        beginPoint = onePoint + min_projection * normal;
        endPoint = onePoint + max_projection * normal;

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
    Eigen::Vector3f point2(point.x,point.y,point.z);

    // 计算点与直线上的点的向量
    Eigen::Vector3f v = point2 - onePoint;

    // 计算叉积
    Eigen::Vector3f crossProduct = v.cross(normal);

    // 计算叉积的模
    float crossProductMagnitude = crossProduct.norm();

    // 如果叉积的模小于给定阈值，则认为点接近直线
    return crossProductMagnitude <= 0.1;
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
