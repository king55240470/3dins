#include "pointcloudconstructor.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkSelectEnclosedPoints.h>
#include <vtkCubeSource.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkProperty.h>
#include<vtkTransform.h>


PointCloudConstructor::PointCloudConstructor() {}
CEntity* PointCloudConstructor::create(QVector<CEntity*>& entitylist){
    if(m_sourceCloud==nullptr){
        setWrongInformation(SourceCloudNull);
        return nullptr;
    }
    CCylinder* cylinder=nullptr;
    CCone*cone=nullptr;
    CSphere* sphere=nullptr;

    for(int i=0;i<entitylist.size();i++){
        CEntity* entity=entitylist[i];
        if(!entity->IsSelected())continue;
        if(entity->GetUniqueType()==enCylinder){
            cylinder=(CCylinder*)entity;
            break;
        }else if(entity->GetUniqueType()==enCone){
            cone=(CCone*)entity;
            break;
        }else if(entity->GetUniqueType()==enSphere){
            sphere=(CSphere*)entity;
            break;
        }
    }
    if(cylinder!=nullptr){
        return createPointCloud(cylinder,m_sourceCloud);
    }
    if(cone!=nullptr){
        return createPointCloud(cone,m_sourceCloud);
    }
    if(sphere!=nullptr){
        return createPointCloud(sphere,m_sourceCloud);
    }
    setWrongInformation(SourceEntityLess);
    return nullptr;
}

CPointCloud* PointCloudConstructor::createPointCloud(vtkPolyData*ployData,vtkSmartPointer<vtkPolyData> pointsPolydata){
    vtkSmartPointer<vtkSelectEnclosedPoints> selectEnclosedPoints = vtkSmartPointer<vtkSelectEnclosedPoints>::New();
    selectEnclosedPoints->SetInputData(pointsPolydata);
    selectEnclosedPoints->SetSurfaceData(ployData);
    selectEnclosedPoints->Update();
    CPointCloud* pointCloud=new CPointCloud();

    vtkPoints* points = pointsPolydata->GetPoints();

    // 获取点的数量
    vtkIdType numberOfPoints = points->GetNumberOfPoints();
    // 遍历所有点并获取坐标

    // 输出每个点的检测结果
    for (vtkIdType i = 0; i < numberOfPoints; i++)
    {
        //std::cout << "Point " << i << " is inside: " << selectEnclosedPoints->IsInside(i) << std::endl;
        if(selectEnclosedPoints->IsInside(i)){
            double point[3];
            // 获取第 i 个点的坐标
            points->GetPoint(i, point);
            pcl::PointXYZRGB pclPoint(point[0],point[1],point[2],0,0,0);
            pointCloud->m_pointCloud.push_back(pclPoint);
        }
    }

    return pointCloud;
}
CPointCloud* PointCloudConstructor::createPointCloud(vtkPolyData*ployData,pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourceCloud){
   vtkSmartPointer<vtkPolyData> pointsPolydata = getPointsPolydata(sourceCloud);
    return createPointCloud(ployData,pointsPolydata);
}
CPointCloud* PointCloudConstructor::createPointCloud(CSphere* sphere,pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourceCloud){
    vtkSmartPointer<vtkPolyData> pointsPolydata = getPointsPolydata(sourceCloud);
    CPosition pos(sphere->getCenter().x, sphere->getCenter().y, sphere->getCenter().z);
    QVector4D posVec = sphere->GetRefCoord()->m_mat * QVector4D(pos.x, pos.y, pos.z, 1);
    CPosition globalPos(posVec.x(), posVec.y(), posVec.z());

    // 创建球体源
    auto m_sphere = vtkSmartPointer<vtkSphereSource>::New();
    m_sphere->SetCenter(globalPos.x, globalPos.y, globalPos.z);
    m_sphere->SetRadius(sphere->getDiameter() / 2);
    m_sphere->SetPhiResolution(100);
    m_sphere->SetThetaResolution(100);
    m_sphere->Update();
    auto polyData=m_sphere->GetOutput();
    return createPointCloud(polyData,pointsPolydata);
}
CPointCloud* PointCloudConstructor::createPointCloud(CCylinder* cylinder,pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourceCloud){
    vtkSmartPointer<vtkPolyData> pointsPolydata = getPointsPolydata(sourceCloud);
    CPosition pos(cylinder->getBtm_center().x, cylinder->getBtm_center().y, cylinder->getBtm_center().z);
    CPosition globalPos(pos.x, pos.y, pos.z);

    // 创建圆柱体源
    auto m_cylinder = vtkSmartPointer<vtkCylinderSource>::New();
    m_cylinder->SetCenter(globalPos.x, globalPos.y, globalPos.z);
    m_cylinder->SetRadius(cylinder->getDiameter() / 2);
    m_cylinder->SetResolution(100);
    m_cylinder->SetHeight(cylinder->getHeight());

    // 创建变换对象，用于旋转圆柱方向
    vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();

    // 归一化axis并应用到变换
    double axisData[3] = {cylinder->getAxis().x(), cylinder->getAxis().y(), cylinder->getAxis().z()};
    transform->RotateWXYZ(180.0, axisData);
    m_cylinder->Update();

    auto polyData=m_cylinder->GetOutput();
    return createPointCloud(polyData,pointsPolydata);
}
CPointCloud* PointCloudConstructor::createPointCloud(CCone* cone,pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourceCloud){
    vtkSmartPointer<vtkPolyData> pointsPolydata = getPointsPolydata(sourceCloud);
    CPosition pos(cone->getVertex().x, cone->getVertex().y, cone->getVertex().z);
    CPosition globalPos(pos.x, pos.y, pos.z);

    // 创建圆锥源
    auto m_cone = vtkSmartPointer<vtkConeSource>::New();
    m_cone->SetCenter(globalPos.x, globalPos.y, globalPos.z);
    m_cone->SetRadius(cone->getRadian());
    m_cone->SetHeight(cone->getCone_height());
    m_cone->SetResolution(100);

    // 创建变换对象，用于旋转圆柱方向
    vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();

    // 归一化axis作为旋转矩阵，并应用到变换
    double axisData[3] = {cone->getAxis().x(), cone->getAxis().y(), cone->getAxis().z()};
    transform->RotateWXYZ(180.0, axisData);
    auto polyData=m_cone->GetOutput();
    return createPointCloud(polyData,pointsPolydata);
    return nullptr;
}
CPointCloud* PointCloudConstructor::createPointCloud(CPosition center, QVector4D XAxis,QVector4D YAxis,QVector4D ZAxis,double XLength,double YLength,double ZLength,pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourceCloud){


    vtkSmartPointer<vtkPolyData> pointsPolydata = getPointsPolydata(sourceCloud);

    //单位化边向量
    XAxis.normalize();
    YAxis.normalize();
    ZAxis.normalize();

    // 创建 VTK PolyData 对象并设置点
    vtkSmartPointer<vtkPoints> points_cube = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkCellArray> polys = vtkSmartPointer<vtkCellArray>::New();
    //八个顶点
    QVector4D peaks[8];
    QVector4D center_(center.x,center.y,center.z,0);
    int XSymbol=1,YSymbol=1,ZSymbol=1;
    for(int i=0;i<2;i++){
        for(int j=0;j<2;j++){
            for(int k=0;k<2;k++){
                QVector4D temp(0,0,0,0);
                temp=center_+XSymbol*(XLength/2)*XAxis+YSymbol*(YLength/2)*YAxis+ZSymbol*(ZLength/2)*ZAxis;
                peaks[i*4+j*2+k]=temp;
                ZSymbol=-ZSymbol;
            }
            YSymbol=-YSymbol;
        }
        XSymbol=-XSymbol;
    }
    // 定义立方体的顶点
    for(int i=0;i<8;i++){
        points_cube->InsertNextPoint(peaks[i].x(),peaks[i].y(),peaks[i].x());
    }

    // 定义立方体的面
    vtkIdType ids[4];
    ids[0] = 0; ids[1] = 1; ids[2] = 3; ids[3] = 2;
    polys->InsertNextCell(4, ids);
    ids[0] = 4; ids[1] = 5; ids[2] = 7; ids[3] = 6;
    polys->InsertNextCell(4, ids);
    ids[0] = 0; ids[1] = 1; ids[2] = 5; ids[3] = 4;
    polys->InsertNextCell(4, ids);
    ids[0] = 2; ids[1] = 3; ids[2] = 7; ids[3] = 6;
    polys->InsertNextCell(4, ids);
    ids[0] = 0; ids[1] = 4; ids[2] = 6; ids[3] = 2;
    polys->InsertNextCell(4, ids);
    ids[0] = 1; ids[1] = 5; ids[2] = 7; ids[3] = 3;
    polys->InsertNextCell(4, ids);

    // 创建多边形数据
    vtkSmartPointer<vtkPolyData> cube = vtkSmartPointer<vtkPolyData>::New();
    cube->SetPoints(points_cube);
    cube->SetPolys(polys);




    // // 创建一个立方体作为闭合曲面
    // vtkSmartPointer<vtkCubeSource> cubeSource = vtkSmartPointer<vtkCubeSource>::New();
    // cubeSource->SetCenter(center.x,center.y,center.z);
    // cubeSource->SetXLength(XLength);
    // cubeSource->SetYLength(YLength);
    // cubeSource->SetZLength(ZLength);
    // cubeSource->Update();

    // vtkPolyData* cube = cubeSource->GetOutput();

    // 使用 vtkSelectEnclosedPoints 检测点是否在立方体内
    return createPointCloud(cube,pointsPolydata);

}

vtkSmartPointer<vtkPolyData> PointCloudConstructor::getPointsPolydata(pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourceCloud){
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    for (const auto& point : *m_sourceCloud)
    {
        points->InsertNextPoint(point.x, point.y, point.z);
    }
    vtkSmartPointer<vtkPolyData> pointsPolydata = vtkSmartPointer<vtkPolyData>::New();
    pointsPolydata->SetPoints(points);
    return pointsPolydata;
}
void PointCloudConstructor::setSourceCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourceCloud){
    m_sourceCloud=sourceCloud;
}
