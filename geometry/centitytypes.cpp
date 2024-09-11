#include "centitytypes.h"
#include <vtkPoints.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyData.h>
#include <vtkProperty.h>

// 点类的draw()
vtkSmartPointer<vtkActor> CPoint::draw(){
    // 创建点集
    auto point = vtkSmartPointer<vtkPoints>::New();
    point->InsertNextPoint(m_pt.x, m_pt.y, m_pt.z);

    // 创建几何图形容器并设置点
    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(point);

    // 创建映射器
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(polyData);

    // 创建执行器，添加mapper
    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetPointSize(3); // 设置点的大小

    return actor;
}

int CLine::lineCount=0;
int CLine::currentLineId=0;
int CPoint::pointCount=0;
int CCircle::circleCount=0;
int CPlane::plainCount=0;
int CSphere::sphereCount=0;
int CCylinder::cylinderCount=0;
int CCone::coneCount=0;

