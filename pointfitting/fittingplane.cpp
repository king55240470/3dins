#include "pointfitting/fittingplane.h"
#include "pcl/common/distances.h"

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
    pcl::search::KdTree<pcl::PointXYZRGB> kdtree;
    //pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
    kdtree.setInputCloud(cloudptr);

    std::vector<int> pointIdxRadiusSearch;//存储邻域点的索引
    std::vector<float> pointRadiusSquaredDistance; // 用于存储找到的点到查询点的平方距离

    // 搜索给定半径内的点
    if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 3) {
        //动态设置距离阈值（可选）
        // float average_distance = 0.0f;
        // for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
        //     average_distance += std::sqrt(pointRadiusSquaredDistance[i]);
        // }
        // average_distance /= pointIdxRadiusSearch.size();
        // qDebug()<<average_distance;

        pcl::copyPointCloud(*cloudptr, pointIdxRadiusSearch, *cloud_subset);//在邻域点中实现RANSAC算法
        // 创建RANSAC分割对象
        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setMaxIterations(5000);//设置最大迭代次数
        seg.setDistanceThreshold(distance);//设置阈值
        // 提供输入点云
        seg.setInputCloud(cloud_subset);

        // 执行分割
        seg.segment(*inliers, *coefficients);

        //获取平面上的所有点云
        for (int i=0;i<cloudptr->size();i++) {
            if(isPointInPlane(cloudptr->points[i])){
                planeCloud->points.push_back(cloudptr->points[i]);
            }
        }
        planeCloud->width = planeCloud->points.size();
        planeCloud->height = 1;
        planeCloud->is_dense = true;
        //将平面上的点云设置为红色
        for (auto& point : planeCloud->points){
            point.r = 255;
            point.g = 0;
            point.b = 0;
        }

        //获取平面法向量
        // normal=Eigen::Vector3f(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
        // normal.normalize();
        // std::cout << "Plane normal: " << normal.transpose() << std::endl;

        pcl::PCA<pcl::PointXYZRGB> pca;
        pca.setInputCloud(planeCloud);
        //获取主成分方向
        Eigen::Matrix3f eigen_vectors = pca.getEigenVectors();  // 特征向量
        // // 投影到主成分方向
        std::vector<double> proj1, proj2;

        for (const auto& point : planeCloud->points) {
            // 将点投影到主成分方向
            double proj1_val = point.x * eigen_vectors(0, 0) + point.y * eigen_vectors(1, 0) + point.z * eigen_vectors(2, 0);
            double proj2_val = point.x * eigen_vectors(0, 1) + point.y * eigen_vectors(1, 1) + point.z * eigen_vectors(2, 1);
            proj1.push_back(proj1_val);
            proj2.push_back(proj2_val);
        }

        //计算投影的最大值和最小值
        double min_proj1 = *std::min_element(proj1.begin(), proj1.end());
        double max_proj1 = *std::max_element(proj1.begin(), proj1.end());
        double min_proj2 = *std::min_element(proj2.begin(), proj2.end());
        double max_proj2 = *std::max_element(proj2.begin(), proj2.end());

        //计算平面的长宽
        length = max_proj1 - min_proj1;  // 平面在第一个方向上的长度
        width = max_proj2 - min_proj2;   // 平面在第二个方向上的宽度
        std::cout << "Plane length: " << length << ", width: " << width << std::endl;
        //获取长边的方向向量
        length_direction = eigen_vectors.col(0);
        //获取平面法向量
        normal=eigen_vectors.col(2);
        //获取平面中心
        center=pca.getMean();

        return planeCloud;

    } else {
        QMessageBox *messagebox=new QMessageBox();
        messagebox->setText("输入的邻域或距离阈值太小，\n请重新输入！");
        messagebox->setIcon(QMessageBox::Warning);
        messagebox->show();
        messagebox->exec();
        PCL_ERROR("Couldn't find more points within radius\n");
        return nullptr;
    }
}

bool FittingPlane::isPointInPlane(const pcl::PointXYZRGB& point){
    double d = coefficients->values[0] * point.x +
               coefficients->values[1] * point.y +
               coefficients->values[2] * point.z +
               coefficients->values[3];
    return fabs(d) <= radius*0.1;
}

void FittingPlane::setRadius(double rad){
    radius=rad;
}

void FittingPlane::setDistance(double dis){
    distance=dis;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr  FittingPlane::getPlaneCloud(){
    return planeCloud;
}

Eigen::Vector3f FittingPlane::getNormal(){
    return normal;
}

Eigen::Vector4f FittingPlane::getCenter(){
    return center;
}

double FittingPlane::getLength(){
    return length;
}

double FittingPlane::getWidth(){
    return width;
}

Eigen::Vector3f FittingPlane::getLength_Direction(){
    return length_direction;
}
