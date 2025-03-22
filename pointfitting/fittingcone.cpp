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
#include <geometry/centitytypes.h>

#include <QMessageBox>
#include <QDebug>

FittingCone::FittingCone() {
    coneCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud_subset.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

    coefficients.reset(new pcl::ModelCoefficients);
    inliers.reset(new pcl::PointIndices);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr FittingCone::RANSAC(pcl::PointXYZRGB searchPoint,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudptr){

    //清除数据
    cloud_subset->clear();
    coneCloud->clear();

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

        // 检查分割是否成功
        if (inliers->indices.empty()) {
            QMessageBox *messagebox = new QMessageBox();
            messagebox->setText("请修改邻域或距离阈值\nRANSAC found no model!\n");
            messagebox->setIcon(QMessageBox::Warning);
            messagebox->show();
            messagebox->exec();
            PCL_ERROR("RANSAC found no model. Error segmenting the model! No solution found.\n");
            return nullptr;
        }

        //计算圆锥顶点
        topCenter=Eigen::Vector3f(coefficients->values[0], coefficients->values[1], coefficients->values[2]);
        qDebug()<<"center:"<<topCenter.x()<<topCenter.y()<<topCenter.z();

        //计算圆锥轴向量
        normal=Eigen::Vector3f(coefficients->values[3], coefficients->values[4], coefficients->values[5]);
        qDebug()<<"normal:"<<normal.x()<<normal.y()<<normal.z();

        //计算圆锥张开角度（弧度）
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

        // 计算圆锥高度：通过点云中的投影来获得
        double min_proj = std::numeric_limits<float>::max();
        double max_proj = std::numeric_limits<float>::lowest();

        // 对所有点进行法向量投影，得到最小和最大投影值
        for (int i = 0; i < coneCloud->size(); i++) {
            Eigen::Vector3f point(coneCloud->points[i].x, coneCloud->points[i].y, coneCloud->points[i].z);
            double proj = point.dot(normal); // 计算点到圆柱轴的投影
            min_proj = std::min(min_proj, proj);
            max_proj = std::max(max_proj, proj);
        }

        height = max_proj - min_proj; // 高度是投影值的差
        qDebug()<<"height:"<<height;

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

    // 计算点到圆锥顶点的向量
    Eigen::Vector3f pointToVertex (point.x - topCenter.x(),point.y - topCenter.y(),point.z - topCenter.z());

    // 计算法向量和点到顶点向量的夹角
    float cosineAlpha = normal.dot(pointToVertex) / (normal.norm() * pointToVertex.norm());

    // 判断是否满足夹角小于或等于圆锥的开口角度
    return cosineAlpha >= cos(angle);
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

Eigen::Vector3f FittingCone::getTopCenter(){
    return topCenter;
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
