#include "centitytypes.h"
#include "component/vtkwidget.h"

#include <vtkPoints.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyData.h>
#include <vtkLineSource.h>
#include <vtkCellArray.h>
#include <vtkEllipseArcSource.h>
#include <vtkPlaneSource.h>
#include <vtkSphereSource.h>
#include <vtkCylinderSource.h>
#include <vtkConeSource.h>
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

    VtkWidget::addActor(actor);
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
    VtkWidget::addActor(actor);
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
    vtkSmartPointer<vtkPolyData> polyData;
    polyData->SetPoints(points);
    polyData->SetLines(lines);

    // 创建映射器和actor
    vtkSmartPointer<vtkPolyDataMapper> mapper;
    mapper->SetInputData(polyData);
    vtkSmartPointer<vtkActor> actor;
    actor->SetMapper(mapper);

    VtkWidget::addActor(actor);
    return actor;
}

// 平面的draw()
vtkSmartPointer<vtkActor> CPlane::draw(){
    // 创建平面源
    vtkSmartPointer<vtkPlaneSource> planeSource = vtkSmartPointer<vtkPlaneSource>::New();
    planeSource->SetCenter(getCenter().x, getCenter().y, getCenter().z); // 设置平面中心
    // planeSource->SetNormal(CPlane::getNormal().x, CPlane::getNormal().y, CPlane::getNormal().z); // 设置平面法线

    // 设置平面的X和Y分辨率
    planeSource->SetXResolution(50);
    planeSource->SetYResolution(50);

    // 创建映射器
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(planeSource->GetOutputPort());

    // 创建执行器
    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);

    VtkWidget::addActor(actor);
    return actor;
}

// 球类的draw()
vtkSmartPointer<vtkActor> CSphere::draw(){
    // 创建球体圆
    auto sphere = vtkSmartPointer<vtkSphereSource>::New();
    sphere->SetCenter(CSphere::getCenter().x, CSphere::getCenter().y, CSphere::getCenter().z);
    sphere->SetRadius(CSphere::getDiameter() / 2);

    // 创建映射器
    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(sphere->GetOutputPort());

    // 创建执行器
    auto actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);

    VtkWidget::addActor(actor);
    return actor;
}

// 圆柱的draw()
vtkSmartPointer<vtkActor> CCylinder::draw(){
    // 创建圆柱体源
    auto cylinder = vtkSmartPointer<vtkCylinderSource>::New();
    cylinder->SetCenter(getBtm_center().x, getBtm_center().y, getBtm_center().z);
    cylinder->SetRadius(getDiameter() / 2);

    // 创建映射器
    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(cylinder->GetOutputPort());

    // 创建执行器
    auto actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);

    VtkWidget::addActor(actor);
    return actor;
}

// 圆锥的draw()
vtkSmartPointer<vtkActor> CCone::draw(){
    // 创建圆锥源
    auto cone = vtkSmartPointer<vtkConeSource>::New();
    cone->SetCenter(getVertex().x, getVertex().y, getVertex().z);
    cone->SetRadius(getRadian());
    cone->SetHeight(getCone_height());

    // 创建映射器
    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(cone->GetOutputPort());

    // 创建执行器
    auto actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);

    VtkWidget::addActor(actor);
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
void CCircle::SetDiameter(double d)
{
    m_d = d;
}
void CCircle::SetCenter(CPosition pt)
{
    m_pt.x = pt.x;
    m_pt.y = pt.y;
    m_pt.z = pt.z;

}

CPosition CCircle::getCenter()
{
    return m_pt;
}

double CCircle::getDiameter()
{
    return m_d;
}
int CCircle::getId()
{
    return currentCircleId;
}
CPosition CLine::getEnd() const
{
    return end;
}

void CLine::setEnd(const CPosition &newEnd)
{
    end = newEnd;
}

CPosition CLine::getBegin() const
{
    return begin;
}

void CLine::setBegin(const CPosition &newBegin)
{
    begin = newBegin;
}

QVector4D CPlane::getNormal() const
{
    return normal;
}

void CPlane::setNormal(const QVector4D &newNormal)
{
    normal = newNormal;
}

QVector4D CPlane::getDir_long_edge() const
{
    return dir_long_edge;
}

void CPlane::setDir_long_edge(const QVector4D &newDir_long_edge)
{
    dir_long_edge = newDir_long_edge;
}

double CPlane::getLength() const
{
    return length;
}

void CPlane::setLength(double newLength)
{
    length = newLength;
}

double CPlane::getWidth() const
{
    return width;
}

void CPlane::setWidth(double newWidth)
{
    width = newWidth;
}

CPosition CPlane::getCenter() const
{
    return center;
}

void CPlane::setCenter(const CPosition &newCenter)
{
    center = newCenter;
}

double CSphere::getDiameter() const
{
    return diameter;
}

void CSphere::setDiameter(double newDiameter)
{
    diameter = newDiameter;
}

CPosition CSphere::getCenter() const
{
    return center;
}

void CSphere::setCenter(const CPosition &newCenter)
{
    center = newCenter;
}

double CCylinder::getDiameter() const
{
    return diameter;
}

void CCylinder::setDiameter(double newDiameter)
{
    diameter = newDiameter;
}

double CCylinder::getHeight() const
{
    return height;
}

void CCylinder::setHeight(double newHeight)
{
    height = newHeight;
}

CPosition CCylinder::getBtm_center() const
{
    return btm_center;
}

void CCylinder::setBtm_center(const CPosition &newBtm_center)
{
    btm_center = newBtm_center;
}

QVector4D CCylinder::getAxis() const
{
    return axis;
}

void CCylinder::setAxis(const QVector4D &newAxis)
{
    axis = newAxis;
}

double CCone::getRadian() const
{
    return radian;
}

void CCone::setRadian(double newRadian)
{
    radian = newRadian;
}

double CCone::getHeight() const
{
    return height;
}

void CCone::setHeight(double newHeight)
{
    height = newHeight;
}

double CCone::getCone_height() const
{
    return cone_height;
}

void CCone::setCone_height(double newCone_height)
{
    cone_height = newCone_height;
}

CPosition CCone::getVertex() const
{
    return vertex;
}

void CCone::setVertex(const CPosition &newVertex)
{
    vertex = newVertex;
}

QVector4D CCone::getAxis() const
{
    return axis;
}

void CCone::setAxis(const QVector4D &newAxis)
{
    axis = newAxis;
}
