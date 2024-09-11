#include "centitytypes.h"
#include "component/vtkpresetwidget.h"

#include <vtkPoints.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyData.h>
#include <vtkLineSource.h>
#include <vtkCellArray.h>
#include <vtkEllipseArcSource.h>
#include <vtkEllipseArcSource.h>
#include <vtkProperty.h>

// 点类的draw()
vtkSmartPointer<vtkActor> CPoint::draw(){
    // 创建点集
    auto point = vtkSmartPointer<vtkPoints>::New();
    point->InsertNextPoint(m_pt.x, m_pt.y, m_pt.z);

    // 创建几何图形容器并设置点集
    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(point);

    // 创建映射器
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(polyData);

    // 创建执行器，添加mapper
    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetPointSize(1); // 设置点的大小

    VtkPresetWidget::addActor(actor);
    return actor;
}

// 线类的draw
vtkSmartPointer<vtkActor> CLine::draw(){
    // 创建点集，并插入定义线的两个点
    auto points = vtkSmartPointer<vtkPoints>::New();
    points->InsertNextPoint(begin.x, begin.y, begin.z);
    points->InsertNextPoint(end.x, end.y, end.z);

    // 创建线源
    auto lines = vtkSmartPointer<vtkCellArray>::New();
    vtkIdType line[2] = {0, 1}; // 索引从0开始
    lines->InsertNextCell(2, line); // 插入一条包含两个顶点的线

    // 创建几何图形容器并设置点和线
    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);
    polyData->SetLines(lines);

    // 创建映射器
    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(polyData);

    // 创建执行器
    auto actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);

    // 添加到渲染窗口中
    VtkPresetWidget::addActor(actor);
    return actor;
}


// 圆类的draw()
vtkSmartPointer<vtkActor> CCircle::draw(){
    // 创建圆上的点集
    vtkSmartPointer<vtkPoints> points;
    const int numPoints = 100; // 圆的点数，更多点数会更平滑
    const double radius = getDiameter()/2; // 圆的半径
    for (int i = 0; i < numPoints; ++i)
    {
        double theta = 2.0 * vtkMath::Pi() * static_cast<double>(i) / static_cast<double>(numPoints);
        double x = radius * cos(theta);
        double y = radius * sin(theta);
        points->InsertNextPoint(x, y, 0.0); // Z坐标设为0
    }

    // 创建一个线源来表示圆的线（多段线）
    vtkSmartPointer<vtkCellArray> lines;
    vtkIdType pointIds[2];
    for (int i = 0; i < numPoints - 1; ++i)
    {
        pointIds[0] = i;
        pointIds[1] = i + 1;
        lines->InsertNextCell(2, pointIds);
    }
    // 闭环：将最后一个点与第一个点连接起来
    pointIds[0] = numPoints - 1;
    pointIds[1] = 0;
    lines->InsertNextCell(2, pointIds);

    // 创建PolyData并设置点和线
    vtkNew<vtkPolyData> polyData;
    polyData->SetPoints(points);
    polyData->SetLines(lines);

    // 创建映射器和actor
    vtkSmartPointer<vtkPolyDataMapper> mapper;
    mapper->SetInputData(polyData);
    vtkSmartPointer<vtkActor> actor;
    actor->SetMapper(mapper);

    VtkPresetWidget::addActor(actor);
    return actor;
}

// 平面的draw()



int CLine::lineCount=0;
int CLine::currentLineId=0;
int CPoint::pointCount=0;
int CCircle::circleCount=0;
int CPlane::plainCount=0;
int CSphere::sphereCount=0;
int CCylinder::cylinderCount=0;
int CCone::coneCount=0;

