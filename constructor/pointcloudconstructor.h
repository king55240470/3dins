#ifndef POINTCLOUDCONSTRUCTOR_H
#define POINTCLOUDCONSTRUCTOR_H
#include"geometry/centitytypes.h"
#include"constructor.h"


#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

//绘制vtk的各种封闭曲面
#include <vtkCylinderSource.h>//圆柱体
#include<vtkCubeSource.h>//长方体
#include<vtkConeSource.h>//圆锥
#include<vtkSphereSource.h>//球
#include<vtkSelectEnclosedPoints.h>//圈中点的算法




class PointCloudConstructor:public Constructor
{
private:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_sourceCloud;
public:
    PointCloudConstructor();
    CEntity* create(QVector<CEntity*>& entitylist)override;
    CPointCloud* createPointCloud(vtkPolyData*ployData,pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourceCloud);
    CPointCloud* createPointCloud(CSphere* sphere,pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourceCloud);
    CPointCloud* createPointCloud(CCylinder* cylinder,pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourceCloud);
    CPointCloud* createPointCloud(CCone* cone,pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourceCloud);
    CPointCloud* createPointCloud(CPosition center, QVector4D XAxis,QVector4D YAxis,QVector4D ZAxis,double XLength,double YLength,double ZLength,pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourceCloud);
    CPointCloud* createPointCloud(vtkPolyData*ployData,vtkSmartPointer<vtkPolyData> pointsPolydata);
    void setSourceCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourceCloud);
    vtkSmartPointer<vtkPolyData> getPointsPolydata(pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourceCloud);
    CPointCloud* createPointCloud(CCuboid* cuboid,pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourceCloud);

};

#endif // POINTCLOUDCONSTRUCTOR_H
