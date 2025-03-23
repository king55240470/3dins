#include "fittingcylinder.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/common/centroid.h>
#include <pcl/common/pca.h>
#include <QDebug>
#include <QMessageBox>
#include <geometry/centitytypes.h>

FittingCylinder::FittingCylinder() {
    cylinderCloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud_subset.reset(new pcl::PointCloud<pcl::PointXYZRGB>);

    coefficients.reset(new pcl::ModelCoefficients);
    inliers.reset(new pcl::PointIndices);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr FittingCylinder::RANSAC(pcl::PointXYZRGB searchPoint,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudptr){

    //清除数据
    cloud_subset->clear();
    cylinderCloud->clear();

    //创建KD树用于邻域搜索
    pcl::search::KdTree<pcl::PointXYZRGB> kdtree;
    kdtree.setInputCloud(cloudptr);

    std::vector<int> pointIdxRadiusSearch;//存储邻域点的索引
    std::vector<float> pointRadiusSquaredDistance; // 用于存储找到的点到查询点的平方距离

    // 搜索给定半径内的点
    if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 4) {

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
        seg.setNormalDistanceWeight(0.01);//设置法向量权重
        seg.setMaxIterations(10000);//设置最大迭代次数
        seg.setDistanceThreshold(distance);//设置阈值
        seg.setInputNormals(normals);
        seg.setInputCloud(cloud_subset);

        // 执行分割
        seg.segment(*inliers, *coefficients);

        // 计算圆柱轴线上某个点
        one_center=Eigen::Vector3f(coefficients->values[0], coefficients->values[1], coefficients->values[2]);

        // 计算圆柱法向量（圆柱轴的方向）
        normal=Eigen::Vector3f(coefficients->values[3], coefficients->values[4], coefficients->values[5]);
        normal.normalize(); // 确保法向量是单位向量

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

        // 计算圆柱直径
        diameter = 2 * coefficients->values[6];

        // 计算圆柱高度：通过点云中的投影来获得
        double min_proj = std::numeric_limits<float>::max();
        double max_proj = std::numeric_limits<float>::lowest();

        // 对所有点进行法向量投影，得到最小和最大投影值
        for (int i = 0; i < cylinderCloud->size(); i++) {
            Eigen::Vector3f point(cylinderCloud->points[i].x, cylinderCloud->points[i].y, cylinderCloud->points[i].z);
            double proj = point.dot(normal); // 计算点到圆柱轴的投影
            //min_proj = std::min(min_proj, proj);
            if(proj<min_proj){
                min_proj=proj;
                bottomCenter=point;
            }
            max_proj = std::max(max_proj, proj);
        }

        height = max_proj - min_proj; // 高度是投影值的差

        //计算圆柱中心
        CPosition P(bottomCenter.x(),bottomCenter.y(),bottomCenter.z());
        CPosition end(one_center.x(),one_center.y(),one_center.z());
        Eigen::Vector3f m_end(one_center.x(),one_center.y(),one_center.z());
        Eigen::Vector3f m_begin=m_end+normal*10;
        CPosition begin(m_begin.x(),m_begin.y(),m_begin.z());

        double ABx = end.x - begin.x;
        double ABy = end.y - begin.y;
        double ABz = end.z - begin.z;

        double APx = P.x - begin.x;
        double APy = P.y - begin.y;
        double APz = P.z - begin.z;

        // 计算 AB 的平方长度
        double AB_squared = ABx * ABx + ABy * ABy + ABz * ABz;

        // 计算点 P 在 AB 上的投影比例 t
        double t = (ABx * APx + ABy * APy + ABz * APz) / AB_squared;

        // 投影落在线段 AB 上
        CPosition projection = {
            begin.x + t * ABx,
            begin.y + t * ABy,
            begin.z + t * ABz
        };
        Eigen::Vector3f m_btm(projection.x,projection.y,projection.z);

        center=(height/2.0f)*normal+m_btm;

        if(height<=0){
            QMessageBox *messagebox=new QMessageBox();
            messagebox->setText("输入的邻域或距离阈值太小，\n请重新输入！");
            messagebox->setIcon(QMessageBox::Warning);
            messagebox->show();
            messagebox->exec();
            PCL_ERROR("Height < 0!\n");
            return nullptr;
        }

        return cylinderCloud;

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

bool FittingCylinder::isPointInCylinder(const pcl::PointXYZRGB& point){
    double r = coefficients->values[6];//圆柱半径

    // 将目标点转为 Eigen 向量
    Eigen::Vector3f p(point.x, point.y, point.z);

    // 计算从轴点到目标点的向量
    Eigen::Vector3f v = p - one_center;

    // 计算点到轴线的距离
    Eigen::Vector3f cross_product = v.cross(normal);//v和normal的叉积
    double distance = cross_product.norm() / normal.norm();  // 点到轴线的垂直距离

    // 判断点是否在圆柱表面
    return fabs(distance - r) <= radius;
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
