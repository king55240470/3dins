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
