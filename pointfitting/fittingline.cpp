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
#include <pcl/features/principal_curvatures.h>

#include <QMessageBox>

#include <future>

FittingLine::FittingLine() {
    //创建点云智能指针
    lineCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud_subset.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    tempCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

    coefficients.reset(new pcl::ModelCoefficients);
    inliers.reset(new pcl::PointIndices);

}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr FittingLine::RANSAC(pcl::PointXYZRGB searchPoint,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudptr){

    linePoint=Eigen::Vector3f(searchPoint.x,searchPoint.y,searchPoint.z);//转换searchPoint的类型

    for (int i=0;i<cloudptr->size();i++) {
        if (euclideanDistance(cloudptr->points[i], searchPoint) < distance) {
            cloud_subset->push_back(cloudptr->points[i]);
            indexs.push_back(i);
        }
    }

    if(cloud_subset->size()>=2){
        // // 直接循环，效率低下，使用下面的方法
        // // 遍历每个点，得到点云曲率
        // for (int i = 0; i < cloudptr->size(); i++) {
        //     double curvature=calculateCurvature(cloudptr, i, 0.1);
        //     curvatures.push_back(curvature);
        //     averageCurvature += curvature;
        // }
        // averageCurvature /= cloudptr->size();

        // //使用std::async创建异步任务，提升运行速度
        // int num_points = cloudptr->size();
        // curvatures.resize(num_points);
        // std::vector<std::future<double>> futures;

        // for (int i = 0; i < num_points; ++i) {
        //     futures.emplace_back(std::async(std::launch::async, &FittingLine::calculateCurvature, this, cloudptr, i, 0.1));
        // }

        // for (int i = 0; i < num_points; ++i) {
        //     curvatures[i] = futures[i].get();
        //     averageCurvature += curvatures[i];
        // }
        //  averageCurvature /= num_points;

        int num_points = cloudptr->size();
        curvatures.resize(num_points);
        averageCurvature = 0.0;

        // 创建线程池
        size_t numThreads = std::thread::hardware_concurrency();
        ThreadPool pool(numThreads);

        // 分块处理点云
        const int blockSize = 100000;
        for (int blockStart = 0; blockStart < num_points; blockStart += blockSize) {
            int blockEnd = std::min(blockStart + blockSize, num_points);
            pool.enqueue([this, cloudptr, blockStart, blockEnd] {
                for (int i = blockStart; i < blockEnd; ++i) {
                    curvatures[i] = calculateCurvature(cloudptr, i, 0.1);
                    qDebug()<<i;
                    std::lock_guard<std::mutex> lock(this->mtx);  // 使用 mutex 保护共享资源
                    averageCurvature += curvatures[i];
                }
            });
        }

        // 等待所有任务完成
        pool.enqueue([] {});
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        averageCurvature /= num_points;

        for (int i = 0; i < cloud_subset->size(); i++) {
            int index = indexs[i];
            if (curvatures[index] > averageCurvature) {
                tempCloud->push_back(cloudptr->points[index]);
            }
        }
    }else{
        QMessageBox *messagebox=new QMessageBox();
        messagebox->setText("输入的距离阈值太小，\n请重新输入！");
        messagebox->setIcon(QMessageBox::Warning);
        messagebox->show();
        messagebox->exec();
        PCL_ERROR("Couldn't find more points within distance\n");
        return nullptr;
    }

    if(tempCloud->size()>=2){

        // 计算点云的质心
        Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
        for (const auto& point : *cloud_subset) {
            centroid.x() += point.x;
            centroid.y() += point.y;
            centroid.z() += point.z;
        }
        centroid /= static_cast<double>(cloud_subset->size());
        onePoint=centroid;

        // 构建协方差矩阵
        Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
        for (const auto& point : *cloud_subset) {
            Eigen::Vector3f diff = Eigen::Vector3f(point.x - centroid.x(),point.y - centroid.y(),point.z - centroid.z());
            cov += diff * diff.transpose();
        }
        cov /= static_cast<double>(cloud_subset->size());

        // 计算协方差矩阵的特征值和特征向量
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig(cov);
        normal = eig.eigenvectors().col(2); // 最大特征值对应的特征向量


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
            float projection = (point.getVector3fMap() - linePoint).dot(normal);

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

        beginPoint = linePoint + min_projection * normal;
        endPoint = linePoint + max_projection * normal;

        // 打印调试信息
        qDebug() << "Begin Point: " << beginPoint.x() << ", " << beginPoint.y() << ", " << beginPoint.z();
        qDebug() << "End Point: " << endPoint.x() << ", " << endPoint.y() << ", " << endPoint.z();

        return lineCloud;

    }else{
        QMessageBox *messagebox=new QMessageBox();
        messagebox->setText("输入的距离阈值太小，\n请重新输入！");
        messagebox->setIcon(QMessageBox::Warning);
        messagebox->show();
        messagebox->exec();
        PCL_ERROR("Couldn't find more points within distance\n");
        return nullptr;
    }
}

// 计算两个点之间的欧几里得距离
double FittingLine::euclideanDistance(const pcl::PointXYZRGB& p1, const pcl::PointXYZRGB& p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    double dz = p1.z - p2.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// 计算指定索引点的曲率
double FittingLine::calculateCurvature(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& pointCloud, int index, double radius) {
    // 获取指定索引的点
    pcl::PointXYZRGB queryPoint = pointCloud->points[index];
    std::vector<pcl::PointXYZRGB> neighbors;

    // 找出邻域内的点
    for (const auto& point : pointCloud->points) {
        if (euclideanDistance(point, queryPoint) < radius) {
            neighbors.push_back(point);
        }
    }

    // 计算邻域点集的质心
    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (const auto& neighbor : neighbors) {
        centroid.x() += neighbor.x;
        centroid.y() += neighbor.y;
        centroid.z() += neighbor.z;
    }
    centroid /= neighbors.size();

    // 计算协方差矩阵
    Eigen::Matrix3d covMatrix = Eigen::Matrix3d::Zero();
    for (const auto& neighbor : neighbors) {
        Eigen::Vector3d diff(neighbor.x - centroid.x(), neighbor.y - centroid.y(), neighbor.z - centroid.z());
        covMatrix += diff * diff.transpose();
    }
    covMatrix /= neighbors.size();

    // 求解协方差矩阵的特征值
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(covMatrix);
    Eigen::Vector3d eigenvalues = eig.eigenvalues();
    eigenvalues = eigenvalues.array().abs();  // 取绝对值

    // 将特征值复制到 std::vector 中进行排序
    std::vector<double> sortedEigenvalues;
    for (int i = 0; i < 3; ++i) {
        sortedEigenvalues.push_back(eigenvalues(i));
    }
    std::sort(sortedEigenvalues.begin(), sortedEigenvalues.end());

    // 计算曲率
    double curvature = sortedEigenvalues[0] / (sortedEigenvalues[0] + sortedEigenvalues[1] + sortedEigenvalues[2]);
    return curvature;
}

bool FittingLine::isPointInLine(const pcl::PointXYZRGB& point){
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
    //radius=rad;
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
