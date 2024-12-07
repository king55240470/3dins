#include "fittingcone.h"

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
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_cone.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <QMessageBox>
#include <QDebug>

FittingCone::FittingCone() {
    coneCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud_subset.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

   // model.reset(new pcl::SampleConsensusModelCone<pcl::PointXYZRGB, pcl::Normal>(cloud_subset));

    coefficients.reset(new pcl::ModelCoefficients);
    inliers.reset(new pcl::PointIndices);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr FittingCone::RANSAC(pcl::PointXYZRGB searchPoint,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudptr){
    //创建KD树用于邻域搜索
    pcl::search::KdTree<pcl::PointXYZRGB> kdtree;
    kdtree.setInputCloud(cloudptr);

    std::vector<int> pointIdxRadiusSearch;//存储邻域点的索引
    std::vector<float> pointRadiusSquaredDistance; // 用于存储找到的点到查询点的平方距离

    // 搜索给定半径内的点
    if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 3) {

        pcl::copyPointCloud(*cloudptr, pointIdxRadiusSearch, *cloud_subset);

        pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
        tree->setInputCloud(cloud_subset);
        ne.setInputCloud(cloud_subset);
        ne.setSearchMethod(tree);
        ne.setKSearch(50);
        ne.compute(*normals);

        if(normals->size()!=cloud_subset->size()){
            QMessageBox *messagebox=new QMessageBox();
            messagebox->setText("输入的邻域太小，\n请重新输入！\n");
            messagebox->setIcon(QMessageBox::Warning);
            messagebox->show();
            messagebox->exec();
            PCL_ERROR("normals->size()!=cloud_subset->size()!\n");
            return nullptr;
        }

        pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_CONE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setNormalDistanceWeight(0.01);//设置法向量权重
        seg.setMaxIterations(10000);//设置最大迭代次数
        seg.setDistanceThreshold(distance);//设置阈值
        seg.setInputNormals(normals);
        seg.setInputCloud(cloud_subset);

        //执行分割
        seg.segment(*inliers, *coefficients);

        //计算圆锥顶点
        topCenter=Eigen::Vector3f(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
        qDebug()<<"center:"<<topCenter.x()<<topCenter.y()<<topCenter.z();

        //计算圆锥轴向量
        normal=Eigen::Vector3f(coefficients->values[3], coefficients->values[4], coefficients->values[5]);
        qDebug()<<"normal:"<<normal.x()<<normal.y()<<normal.z();

        //计算圆锥半角
        angle=2*coefficients->values[6];
        qDebug()<<"angle:"<<angle;

        //获取圆锥上的所有点云
        for (int i=0;i<cloudptr->size();i++) {
            if(isPointInCone(cloudptr->points[i])){
                coneCloud->points.push_back(cloudptr->points[i]);
            }
        }
        coneCloud->width = coneCloud->points.size();
        coneCloud->height = 1;
        coneCloud->is_dense = true;
        //将圆柱上的点云设置为红色
        for (auto& point : coneCloud->points){
            point.r = 255;
            point.g = 0;
            point.b = 0;
        }

        if(coneCloud->empty()){
            QMessageBox *messagebox=new QMessageBox();
            messagebox->setText("请修改邻域或距离阈值，\n有可能是邻域太小，\n或者距离阈值太大，\nConeCloud is empty!\n");
            messagebox->setIcon(QMessageBox::Warning);
            messagebox->show();
            messagebox->exec();
            PCL_ERROR("ConeCloud is empty!\n");
            return nullptr;
        }

        // 计算圆锥的高度：通过点云中的投影来获得
        double min_proj = std::numeric_limits<float>::max();
        double max_proj = std::numeric_limits<float>::lowest();

        // 对所有点进行法向量投影，得到最小和最大投影值
        for (int i = 0; i < coneCloud->size(); i++) {
            Eigen::Vector3f point(coneCloud->points[i].x, coneCloud->points[i].y, coneCloud->points[i].z);
            double proj = point.dot(normal); // 计算点到圆柱轴的投影
            min_proj = std::min(min_proj, proj);
            max_proj = std::max(max_proj, proj);
        }

        // qDebug()<<"center:"<<topCenter.x()<<topCenter.y()<<topCenter.z();

        height = max_proj - min_proj; // 高度是投影值的差
        qDebug()<<"height:"<<height;

        // //计算圆锥半角
        // angle=coefficients->values[6];
        // //计算圆锥张开的角度
        // angle = 2 * angle * 180.0 / M_PI;
        // qDebug()<<"angle:"<<angle;

        // //获取圆锥上的所有点云
        // for (int i=0;i<cloudptr->size();i++) {
        //     if(isPointInCone(cloudptr->points[i])){
        //         coneCloud->points.push_back(cloudptr->points[i]);
        //     }
        // }
        // // for (int index : inliers->indices) {
        // //     coneCloud->push_back(cloudptr->points[index]);
        // // }
        // coneCloud->width = coneCloud->points.size();
        // coneCloud->height = 1;
        // coneCloud->is_dense = true;
        // //将圆柱上的点云设置为红色
        // for (auto& point : coneCloud->points){
        //     point.r = 255;
        //     point.g = 0;
        //     point.b = 0;
        // }

        // // 使用PCA拟合圆锥轴
        // // pcl::PCA<pcl::PointXYZRGB> pca;
        // // pca.setInputCloud(coneCloud);
        // // Eigen::Matrix3f eigen_vectors = pca.getEigenVectors();
        // // // 获取主成分方向作为圆锥的轴
        // // normal = eigen_vectors.col(2);  // Z轴是主轴
        // // normal.normalize();
        // // std::cout<<"normal2:"<<normal;

        // // 对所有点进行法向量投影，得到最小和最大投影值
        // // for (int i = 0; i < coneCloud->size(); i++) {
        // //     Eigen::Vector3f point(coneCloud->points[i].x, coneCloud->points[i].y, coneCloud->points[i].z);
        // //     double proj = point.dot(normal); // 计算点到圆柱轴的投影
        // //     min_proj = std::min(min_proj, proj);
        // //     //max_proj = std::max(max_proj, proj);
        // //     if(proj>max_proj){
        // //         max_proj=proj;
        // //         topCenter=point;
        // //     }
        // // }

        // // qDebug()<<"center:"<<topCenter.x()<<topCenter.y()<<topCenter.z();

        // // height = max_proj - min_proj; // 高度是投影值的差
        // // qDebug()<<"height:"<<height;

        //计算圆锥中心
        center = topCenter - (normal * (height / 2.0));

        if(height<=0){
            QMessageBox *messagebox=new QMessageBox();
            messagebox->setText("输入的邻域或距离阈值太小，\n请重新输入！\nHeight < 0!");
            messagebox->setIcon(QMessageBox::Warning);
            messagebox->show();
            messagebox->exec();
            PCL_ERROR("Height < 0!\n");
            return nullptr;
        }

        return coneCloud;

    }else{
        QMessageBox *messagebox=new QMessageBox();
        messagebox->setText("输入的邻域或距离阈值太小，\n请重新输入！\nCouldn't find more points within radius");
        messagebox->setIcon(QMessageBox::Warning);
        messagebox->show();
        messagebox->exec();
        PCL_ERROR("Couldn't find more points within radius\n");
        return nullptr;
    }
}

bool FittingCone::isPointInCone(const pcl::PointXYZRGB& point){

    Eigen::Vector3f pointVec(point.x - topCenter.x(), point.y - topCenter.y(), point.z - topCenter.z());

    double coneHeight = pointVec.dot(normal);  // 投影到圆锥的轴上（即高度）

    double coneRadius = tan(angle) * coneHeight;  // 圆锥半径

    // 计算点与轴的垂直距离
    double perpendicularDistance = std::sqrt(std::pow(point.x - topCenter.x(), 2) +
                                            std::pow(point.y - topCenter.y(), 2));

    return perpendicularDistance <= coneRadius;
}

void FittingCone::setRadius(double rad){
    radius=rad;
}

void FittingCone::setDistance(double dis){
    distance=dis;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr FittingCone::getConeCloud(){
    return coneCloud;
}

Eigen::Vector3f FittingCone::getCenter(){
    return center;
}

Eigen::Vector3f FittingCone::getNormal(){
    return normal;
}

double FittingCone::getAngle(){
    return angle;
}

double FittingCone::getHeight(){
    return height;
}
