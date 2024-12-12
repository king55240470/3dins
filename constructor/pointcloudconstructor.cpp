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


#include <vtkSmartPointer.h>
#include <vtkCubeSource.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>


#include <cmath>


PointCloudConstructor::PointCloudConstructor() {}
CEntity* PointCloudConstructor::create(QVector<CEntity*>& entitylist){
    if(m_sourceCloud==nullptr){
        setWrongInformation(SourceCloudNull);
        return nullptr;
    }
    CCylinder* cylinder=nullptr;
    CCone*cone=nullptr;
    CSphere* sphere=nullptr;
    CCuboid* cuboid=nullptr;
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
        }else if(entity->GetUniqueType()==enCuboid){
            cuboid=(CCuboid*)entity;
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
    if(cuboid!=nullptr){
        return createPointCloud(cuboid,m_sourceCloud);
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
CPointCloud* PointCloudConstructor::createPointCloud(CCylinder* m_cylinder,pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourceCloud){
    vtkSmartPointer<vtkPolyData> pointsPolydata = getPointsPolydata(sourceCloud);
    CPosition pos( m_cylinder->getBtm_center().x, m_cylinder->getBtm_center().y, m_cylinder->getBtm_center().z);
    QVector4D posVec = m_cylinder->GetRefCoord()->m_mat * QVector4D(pos.x, pos.y, pos.z, 1);
    CPosition globalPos(posVec.x(), posVec.y(), posVec.z());

    // 创建圆柱体源
    auto cylinder = vtkSmartPointer<vtkCylinderSource>::New();
    cylinder->SetCenter(globalPos.x, globalPos.y, globalPos.z);
    cylinder->SetRadius(m_cylinder->getDiameter() / 2);
    cylinder->SetResolution(100);
    cylinder->SetHeight(m_cylinder->getHeight());

    // 创建变换对象，用于旋转圆柱方向
    vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
    // 默认轴向量为y轴
    QVector3D yAxis(0, 1, 0);
    // 归一化axis，作为目标轴向量
    QVector3D targetAxis(m_cylinder->getAxis());
    targetAxis.normalize();

    // 计算旋转角度
    auto dotProduct = QVector3D::dotProduct(yAxis, targetAxis);
    double angle = acos(dotProduct) * 180.0 / vtkMath::Pi(); // 将弧度转换为度

    // 计算旋转轴
    QVector3D rotationAxis = QVector3D::crossProduct(yAxis, targetAxis);
    rotationAxis.normalize();

    // 应用旋转
    transform->Translate(globalPos.x, globalPos.y, globalPos.z); // 将旋转中心移动到圆柱体中心
    transform->RotateWXYZ(angle, rotationAxis.x(), rotationAxis.y(), rotationAxis.z());
    transform->Translate(-globalPos.x, -globalPos.y, -globalPos.z); // 将旋转中心移回原点

    vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkTransformPolyDataFilter::New();
    transformFilter->SetInputConnection(cylinder->GetOutputPort());
    transformFilter->SetTransform(transform);
    transformFilter->Update();

    // 获取变换后的 vtkPolyData
    vtkSmartPointer<vtkPolyData> transformedPolyData = transformFilter->GetOutput();

    return createPointCloud(transformedPolyData,pointsPolydata);
}
CPointCloud* PointCloudConstructor::createPointCloud(CCone *m_cone, pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourceCloud){
     vtkSmartPointer<vtkPolyData> pointsPolydata = getPointsPolydata(sourceCloud);


    CPosition pos(m_cone->getVertex().x, m_cone->getVertex().y, m_cone->getVertex().z);
    QVector4D posVec = m_cone->GetRefCoord()->m_mat * QVector4D(pos.x, pos.y, pos.z, 1);
    QVector4D center=posVec-m_cone->getAxis()*(m_cone->getHeight()/2.0);
    CPosition globalPos(center.x(), center.y(), center.z());

    // 创建圆锥源
    auto cone = vtkSmartPointer<vtkConeSource>::New();
    cone->SetAngle(m_cone->getRadian()*180/M_PI);
    cone->SetCenter(globalPos.x, globalPos.y, globalPos.z);
    cone->SetRadius(tan(m_cone->getRadian()/2)*m_cone->getHeight());
    cone->SetDirection(m_cone->getAxis()[0], m_cone->getAxis()[1], m_cone->getAxis()[2]); // 设置轴向量
    cone->SetHeight(m_cone->getHeight());
    cone->SetResolution(100);
    cone->CappingOn();

    return createPointCloud(cone->GetOutput(),pointsPolydata);
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
        points_cube->InsertNextPoint(peaks[i].x(),peaks[i].y(),peaks[i].z());
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
CPointCloud* PointCloudConstructor::createPointCloud(CCuboid* m_cuboid,pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourceCloud){
    if(m_cuboid!=nullptr){

        CPosition pos(m_cuboid->getCenter().x,m_cuboid->getCenter().y, m_cuboid->getCenter().z);
        QVector4D posVec = m_cuboid->GetRefCoord()->m_mat * QVector4D(pos.x, pos.y, pos.z, 1);
        CPosition globalPos(posVec.x(), posVec.y(), posVec.z());

        // 创建 vtkCubeSource 并设置其属性
        auto cuboid = vtkSmartPointer<vtkCubeSource>::New();
        cuboid->SetCenter(globalPos.x, globalPos.y, globalPos.z); // 设置长方体中心点
        cuboid->SetXLength(m_cuboid->getLength()); // 设置长方体长度
        cuboid->SetYLength(m_cuboid->getWidth());  // 设置长方体宽度
        cuboid->SetZLength(m_cuboid->getHeight()); // 设置长方体高度

        // // 创建变换对象并设置旋转角度
        // vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
        // transform->RotateX(m_cuboid->getAngleX()); // 绕X轴旋转
        // transform->RotateY(m_cuboid->getAngleY()); // 绕Y轴旋转
        // transform->RotateZ(m_cuboid->getAngleZ()); // 绕Z轴旋转

        // // 应用变换到 vtkPolyData
        // vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkTransformPolyDataFilter::New();
        // transformFilter->SetInputConnection(cuboid->GetOutputPort());
        // transformFilter->SetTransform(transform);
        // transformFilter->Update();

        // // 获取变换后的 vtkPolyData
        // vtkSmartPointer<vtkPolyData> transformedPolyData = transformFilter->GetOutput();


        // return createPointCloud(transformedPolyData,sourceCloud);

        vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();

        // 提取法向量
        QVector4D normal = m_cuboid->getNormal(); // 假设 GetNormal() 返回法向量
        double nx = normal.x();
        double ny = normal.y();
        double nz = normal.z();

        // 确保法向量的长度为 1
        double length = std::sqrt(nx * nx + ny * ny + nz * nz);
        if (length > 0.0) {
            nx /= length;
            ny /= length;
            nz /= length;
        }

        // 计算旋转角度和旋转轴
        double rotationAxis[3] = {0.0, 0.0, 1.0}; // 默认旋转轴（Z 轴）
        double rotationAngle = 0.0;               // 默认旋转角度

        // 如果法向量不是 Z 轴方向，则计算旋转轴和角度
        if (nx != 0.0 || ny != 0.0 || nz != 1.0) {
            double zAxis[3] = {0.0, 0.0, 1.0};
            double normalVec[3] = {nx, ny, nz};

            // 计算旋转轴 = Z 轴 × 法向量
            vtkMath::Cross(zAxis, normalVec, rotationAxis);

            // 计算旋转角度 = arccos(Z 轴 · 法向量)
            double dotProduct = vtkMath::Dot(zAxis, normalVec);
            rotationAngle = vtkMath::DegreesFromRadians(std::acos(dotProduct));
        }

        // 应用旋转到 vtkTransform
        transform->RotateWXYZ(rotationAngle, rotationAxis[0], rotationAxis[1], rotationAxis[2]);

        // 应用变换到 vtkPolyData
        vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
        transformFilter->SetInputConnection(cuboid->GetOutputPort());
        transformFilter->SetTransform(transform);
        transformFilter->Update();

        // 获取变换后的 vtkPolyData
        vtkSmartPointer<vtkPolyData> transformedPolyData = transformFilter->GetOutput();

        return createPointCloud(transformedPolyData, sourceCloud);

        // QVector4D xAxis(1,0,0,0),yAxis(0,1,0,0),zAxis(0,0,1,0);
        // double x= M_PI*cuboid->getAngleX()/180;
        // double y= M_PI*cuboid->getAngleY()/180;
        // double z= M_PI*cuboid->getAngleZ()/180;


        // xAxis.setX(cos(y)*cos(z)-sin(y)*sin(z));
        // xAxis.setY(sin(z)*cos(y)+sin(y)*cos(z));
        // xAxis.setZ(-sin(y));


        // yAxis.setX(cos(z)*sin(y)*sin(x)+sin(z)*cos(x));
        // yAxis.setY(sin(x)*sin(y)*sin(z)-cos(z)*cos(x));
        // yAxis.setZ(cos(y)*sin(x));


        // zAxis.setX(cos(z)*sin(y)*cos(x)-sin(z)*sin(x));
        // zAxis.setY(sin(z)*sin(y)*cos(x)+cos(z)*sin(x));
        // zAxis.setZ(cos(y)*cos(x));

        // return createPointCloud(cuboid->getCenter(),xAxis,yAxis,zAxis,cuboid->getLength(),cuboid->getWidth(),cuboid->getHeight(),sourceCloud);
    }
    return nullptr;
}
