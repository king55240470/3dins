#include "centitytypes.h"

#include <vtkPoints.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyData.h>
#include <vtkPointData.h>
#include <vtkUnsignedCharArray.h>
#include <vtkLineSource.h>
#include <vtkLine.h>
#include <vtkCellArray.h>
#include <vtkPlaneSource.h>
#include <vtkSphereSource.h>
#include <vtkCylinderSource.h>
#include <vtkConeSource.h>
#include <vtkProperty.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkPolygon.h>
#include <vtkMath.h>


// 点类的draw()
vtkSmartPointer<vtkActor> CPoint::draw(){
    // 创建点集
    auto points = vtkSmartPointer<vtkPoints>::New();

    // 获取图形在参考坐标系下的坐标(预置时输入的)，并计算得到他在机械坐标系下的位置(全局坐标)
    CPosition pos(m_pt.x, m_pt.y, m_pt.z);
    QVector4D posVec = GetRefCoord()->m_mat * QVector4D(pos.x, pos.y, pos.z, 1);
    CPosition globalPos(posVec.x(), posVec.y(), posVec.z());

    points->InsertNextPoint(globalPos.x, globalPos.y, globalPos.z);

    // 创建几何图形容器并设置点集
    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);

    // 创建一个顶点,用过滤器将提取的点转化为更好观察的图形(glyph)，改善可视化效果
    vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
    glyphFilter->SetInputData(polyData);
    glyphFilter->Update();

    // 创建映射器
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(glyphFilter->GetOutputPort());

    // 创建执行器，添加mapper
    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetPointSize(5); // 设置点的大小
    actor->GetProperty()->SetColor(0, 0, 0);

    return actor;
}

QString CPoint::setentityinfo()
{
    QString infoText = QString("X:%1\nY:%2\nZ:%3").arg(m_pt.x).arg(m_pt.y).arg(m_pt.z);
    return infoText;
}

// 线类的draw
vtkSmartPointer<vtkActor> CLine::draw(){
    // 获取首尾两个点在参考坐标系下的坐标(预置时输入的)，
    // 并计算得到他在机械坐标系下的位置(全局坐标)
    CPosition pos_begin(begin.x, begin.y, begin.z);
    QVector4D posVec_begin = GetRefCoord()->m_mat * QVector4D(pos_begin.x, pos_begin.y, pos_begin.z, 1);
    CPosition glbPos_begin(posVec_begin.x(), posVec_begin.y(), posVec_begin.z());

    CPosition pos_end(end.x, end.y, end.z);
    QVector4D posVec_end = GetRefCoord()->m_mat * QVector4D(pos_end.x, pos_end.y, pos_end.z, 1);
    CPosition glbPos_end(posVec_end.x(), posVec_end.y(), posVec_end.z());

    // 创建点集，并插入定义线的两个点
    auto points = vtkSmartPointer<vtkPoints>::New();
    points->InsertNextPoint(glbPos_begin.x, glbPos_begin.y, glbPos_begin.z);
    points->InsertNextPoint(glbPos_end.x, glbPos_end.y, glbPos_end.z);

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
    actor->GetProperty()->SetColor(0.0, 0.0, 0.0);
    actor->GetProperty()->SetLineWidth(3);

    // 添加到渲染窗口中
    return actor;
}


// 圆类的draw()
vtkSmartPointer<vtkActor> CCircle::draw(){
    // 获取图形在参考坐标系下的坐标(预置时输入的)，并计算得到他在机械坐标系下的位置(全局坐标)
    CPosition pos(m_pt.x, m_pt.y, m_pt.z);
    QVector4D posVec = GetRefCoord()->m_mat * QVector4D(pos.x, pos.y, pos.z, 1);
    CPosition globalPos(posVec.x(), posVec.y(), posVec.z());

    // 创建圆上的点集
    auto points = vtkSmartPointer<vtkPoints>::New();
    const int numPoints = 100; // 圆的点数，更多点数会更平滑
    const double radius = getDiameter()/2; // 圆的半径
    auto center = globalPos;
    for (int i = 0; i < numPoints; ++i)
    {
        double theta = 2.0 * vtkMath::Pi() * static_cast<double>(i) / static_cast<double>(numPoints);
        double x = center.x + radius * cos(theta); // 加上中心x坐标
        double y = center.y + radius * sin(theta); // 加上中心y坐标
        double z = center.z;
        points->InsertNextPoint(x, y, z); // Z坐标设为0
    }

    // 创建一个线源来表示圆的线（多段线）
    auto lines = vtkSmartPointer<vtkCellArray>::New();
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
    auto polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);
    polyData->SetLines(lines);

    // 创建映射器和actor
    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(polyData);

    auto actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(0.0, 0.0, 0.0);
    actor->GetProperty()->SetLineWidth(3);

    return actor;
}

vtkSmartPointer<vtkActor> CPlane::draw(){
    // 获取图形在参考坐标系下的坐标(预置时输入的)，并计算得到他在机械坐标系下的位置(全局坐标)
    CPosition pos(getCenter().x, getCenter().y, getCenter().z);
    //QVector4D posVec = GetRefCoord()->m_mat * QVector4D(pos.x, pos.y, pos.z, 1);
    CPosition globalPos(pos.x, pos.y, pos.z);

    // 创建面——矩形法
    double halfL = getLength() / 2.0;
    double halfW = getWidth() / 2.0;

    QVector4D normalVec = getNormal(); // 获取法向量
    // 将法向量单位化
    double norm_length = sqrt(normalVec.x() * normalVec.x() + normalVec.y() * normalVec.y() + normalVec.z() * normalVec.z());
    QVector4D unitNormal = normalVec / norm_length;

    //现在第一个向量是长边向量
    QVector3D firstPerpVec = dir_long_edge.toVector3D();

    double norm_1 = sqrt(firstPerpVec.x() * firstPerpVec.x() + firstPerpVec.y() * firstPerpVec.y() + firstPerpVec.z() * firstPerpVec.z());
    //这个才是单位化后的长边向量
    QVector3D unitNormal_1 = firstPerpVec / norm_1;

    // 3.计算第二个向量，用normal和firstPerpVec的叉积

    QVector3D secondPerpVec = QVector3D::crossProduct(unitNormal_1,unitNormal.toVector3D());

    //计算四个顶点的全局坐标
    double p1x = globalPos.x + halfW * secondPerpVec.x() - halfL * unitNormal_1.x();
    double p1y = globalPos.y + halfW * secondPerpVec.y() - halfL * unitNormal_1.y();
    double p1z = globalPos.z + halfW * secondPerpVec.z() - halfL * unitNormal_1.z();

    double p2x = globalPos.x + halfW * secondPerpVec.x() + halfL * unitNormal_1.x();
    double p2y = globalPos.y + halfW * secondPerpVec.y() + halfL * unitNormal_1.y();
    double p2z = globalPos.z + halfW * secondPerpVec.z() + halfL * unitNormal_1.z();

    double p3x = globalPos.x - halfW * secondPerpVec.x() + halfL * unitNormal_1.x();
    double p3y = globalPos.y - halfW * secondPerpVec.y() + halfL * unitNormal_1.y();
    double p3z = globalPos.z - halfW * secondPerpVec.z() + halfL * unitNormal_1.z();

    double p4x = globalPos.x - halfW * secondPerpVec.x() - halfL * unitNormal_1.x();
    double p4y = globalPos.y - halfW * secondPerpVec.y() - halfL * unitNormal_1.y();
    double p4z = globalPos.z - halfW * secondPerpVec.z() - halfL * unitNormal_1.z();
    // 向点集插入四个点
    auto points = vtkSmartPointer<vtkPoints>::New();
    points->InsertNextPoint(p1x, p1y, p1z);
    points->InsertNextPoint(p2x, p2y, p2z);
    points->InsertNextPoint(p3x, p3y, p3z);
    points->InsertNextPoint(p4x, p4y, p4z);

    // 得到点集的几何数据
    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);

    // 创建一个vtkCellArray对象来存储多边形
    vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
    // 定义一个四边形（四个顶点），注意VTK中的多边形索引是从0开始的
    vtkIdType verts[4] = {0, 1, 2, 3}; // 这里的0,1,2,3是点的索引
    cells->InsertNextCell(4, verts); // 插入一个包含4个顶点的多边形

    // 设置多边形到polyData
    polyData->SetPolys(cells);

    // 创建一个vtkPolyDataMapper来映射polyData到图形表示
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(polyData);

    // 创建一个vtkActor来表示多边形
    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    // 可以设置演员的属性，比如颜色
    actor->GetProperty()->SetColor(0.7, 0.7, 0.7);

    return actor;
}

// 球类的draw()
vtkSmartPointer<vtkActor> CSphere::draw(){
    // 获取图形在参考坐标系下的坐标(预置时输入的)，并计算得到他在机械坐标系下的位置(全局坐标)
    CPosition pos(getCenter().x, getCenter().y, getCenter().z);
    QVector4D posVec = GetRefCoord()->m_mat * QVector4D(pos.x, pos.y, pos.z, 1);
    CPosition globalPos(posVec.x(), posVec.y(), posVec.z());

    // 创建球体源
    auto sphere = vtkSmartPointer<vtkSphereSource>::New();
    sphere->SetCenter(globalPos.x, globalPos.y, globalPos.z);
    sphere->SetRadius(CSphere::getDiameter() / 2);
    sphere->SetPhiResolution(100);
    sphere->SetThetaResolution(100);

    // 创建映射器
    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(sphere->GetOutputPort());

    // 创建执行器
    auto actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(0.5, 0.5, 0.5);

    return actor;
}

// 圆柱的draw()
vtkSmartPointer<vtkActor> CCylinder::draw(){
    // 获取图形在参考坐标系下的坐标(预置时输入的)，并计算得到他在机械坐标系下的位置(全局坐标)
    CPosition pos(getBtm_center().x, getBtm_center().y, getBtm_center().z);
    QVector4D posVec = GetRefCoord()->m_mat * QVector4D(pos.x, pos.y, pos.z, 1);
    CPosition globalPos(posVec.x(), posVec.y(), posVec.z());

    // 创建圆柱体源
    auto cylinder = vtkSmartPointer<vtkCylinderSource>::New();
    cylinder->SetCenter(globalPos.x, globalPos.y, globalPos.z);
    cylinder->SetRadius(getDiameter() / 2);
    cylinder->SetResolution(100);
    cylinder->SetHeight(getHeight());

    // 创建映射器
    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(cylinder->GetOutputPort());

    // 创建执行器
    auto actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(0.5, 0.5, 0.5);

    return actor;
}

// 圆锥的draw()
vtkSmartPointer<vtkActor> CCone::draw(){
    // 获取图形在参考坐标系下的坐标(预置时输入的)，并计算得到他在机械坐标系下的位置(全局坐标)
    CPosition pos(getVertex().x, getVertex().y, getVertex().z);
    QVector4D posVec = GetRefCoord()->m_mat * QVector4D(pos.x, pos.y, pos.z, 1);
    CPosition globalPos(posVec.x(), posVec.y(), posVec.z());

    // 创建圆锥源
    auto cone = vtkSmartPointer<vtkConeSource>::New();
    cone->SetCenter(globalPos.x, globalPos.y, globalPos.z);
    cone->SetRadius(getRadian());
    cone->SetHeight(getCone_height());
    cone->SetResolution(100);

    // 创建映射器
    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(cone->GetOutputPort());

    // 创建执行器
    auto actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(0.5, 0.5, 0.5);

    return actor;
}

int CPointCloud::pointCloudCount = 0;
vtkSmartPointer<vtkActor> CPointCloud::draw(){
    // 将cloud转换为VTK的点集
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
    colors->SetNumberOfComponents(3);
    colors->SetName("Colors");

    points->SetNumberOfPoints(m_pointCloud.points.size());
    for (size_t i = 0; i < m_pointCloud.points.size(); ++i)
    {
        points->SetPoint(i, m_pointCloud.points[i].x, m_pointCloud.points[i].y, m_pointCloud.points[i].z);
    }

    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);
    polyData->GetPointData()->SetScalars(colors);

    // 创建一个顶点过滤器来生成顶点表示
    vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter = vtkSmartPointer<vtkVertexGlyphFilter>::New();
    glyphFilter->SetInputData(polyData);
    glyphFilter->Update();

    polyData = glyphFilter->GetOutput();

    // 创建映射器并将glyphFilter的几何数据输入
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(polyData);

    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetPointSize(5); // 设置点大小
    if(isFileCloud){
        actor->GetProperty()->SetColor(0.5, 0.5, 0.5);
    }
    else{
        actor->GetProperty()->SetColor(1, 0, 0);
    }


    return actor;
}

// 各种距离的draw
vtkSmartPointer<vtkActor> CDistance::draw(){
    vtkSmartPointer<vtkActor> actor;

    if(isHavePlane)
        actor = pointToPlane();
    else if(isHaveLine)
        actor = pointToLine();
    else {
        actor = pointToCircle();
    }

    return actor;
}

// 绘制点到面的垂线
vtkSmartPointer<vtkActor> CDistance::pointToPlane()
{
    // 将begin转为全局坐标
    CPosition pos_begin(begin.x, begin.y, begin.z);
    QVector4D posVec_begin = GetRefCoord()->m_mat * QVector4D(pos_begin.x, pos_begin.y, pos_begin.z, 1);
    CPosition glbPos_begin(posVec_begin.x(), posVec_begin.y(), posVec_begin.z());

    // 取平面上一点
    CPosition plane_point = plane.getCenter();
    QVector4D posVec_point = GetRefCoord()->m_mat * QVector4D(plane_point.x, plane_point.y, plane_point.z, 1);
    CPosition glbPos_point(posVec_point.x(), posVec_point.y(), posVec_point.z());
    // 取平面法向量
    QVector4D plane_nomal = plane.getNormal();

    // 计算点到平面的距离
    // 点到平面的距离公式: d = |(P - P0) · N| / ||N||
    double distance = getdistanceplane();

    // 计算glbPos_begin在平面上的落点
    CPosition projection;
    projection.x = glbPos_begin.x - distance * plane_nomal.x();
    projection.y = glbPos_begin.y - distance * plane_nomal.y();
    projection.z = glbPos_begin.z - distance * plane_nomal.z();
    QVector4D posVec_pro = GetRefCoord()->m_mat * QVector4D(projection.x, projection.y, projection.z, 1);
    CPosition glb_pro(posVec_pro.x(), posVec_pro.y(), posVec_pro.z());


    // 创建点集，并插入定义线的两个点
    auto points = vtkSmartPointer<vtkPoints>::New();
    points->InsertNextPoint(glbPos_begin.x, glbPos_begin.y, glbPos_begin.z);
    points->InsertNextPoint(glb_pro.x, glb_pro.y, glb_pro.z);

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
    actor->GetProperty()->SetColor(0.0, 0.0, 0.0);
    actor->GetProperty()->SetLineWidth(3);

    return actor;
}

// 绘制点到线的垂线
vtkSmartPointer<vtkActor> CDistance::pointToLine()
{
    // 将begin转为全局坐标
    CPosition pos_begin(begin.x, begin.y, begin.z);
    QVector4D posVec_begin = GetRefCoord()->m_mat * QVector4D(pos_begin.x, pos_begin.y, pos_begin.z, 1);
    CPosition glbPos_begin(posVec_begin.x(), posVec_begin.y(), posVec_begin.z());

    // 取直线首尾两点做方向向量
    CPosition line_begin = line.begin;
    CPosition line_end = line.getEnd();
    QVector4D lineVec_begin = GetRefCoord()->m_mat * QVector4D(line_begin.x, line_begin.y, line_begin.z, 1);
    QVector4D lineVec_end = GetRefCoord()->m_mat * QVector4D(line_end.x, line_end.y, line_end.z, 1);
    CPosition glbline_begin(lineVec_begin.x(), lineVec_begin.y(), lineVec_begin.z());
    CPosition glbline_end(lineVec_end.x(), lineVec_end.y(), lineVec_end.z());

    // 得到方向向量并单位化
    QVector3D lineVec(glbline_begin.x-glbline_end.x, glbline_begin.y-glbline_end.y
                      , glbline_begin.z-glbline_end.z);
    lineVec.normalize();

    // 选取不与直线共线的向量，这里要分别判断三个分量是否为0，来选取法向量
    QVector3D vec(1, 0, 0);
    if (lineVec.x() == 1.0 || lineVec.x() == -1.0) { // 避免与lineDirection共线
        vec = QVector3D(0, 1, 0);
        if (lineVec.y() == 1.0 || lineVec.y() == -1.0) {
            vec = QVector3D(0, 0, 1);
        }
    }

    // 计算垂直向量
    // QVector3D normalVertical = QVector3D::crossProduct(lineVec, vec);

    // 得到点到直线的距离
    // double distance = getdistanceline();
    // 计算从直线起点到点的向量
    QVector3D pointToLineVec(glbPos_begin.x - glbline_begin.x, glbPos_begin.y - glbline_begin.y
                             , glbPos_begin.z - glbline_begin.z);

    // 计算点积 v · w
    double dotProduct = QVector3D::dotProduct(pointToLineVec, lineVec);

    // 计算垂足
    // 垂足计算公式
    // p = p0 + v.w.w / |w|^2
    QVector3D projection = QVector3D(lineVec_begin) + dotProduct * lineVec;

    // 创建点集，并插入定义线的两个点
    auto points = vtkSmartPointer<vtkPoints>::New();
    points->InsertNextPoint(glbPos_begin.x, glbPos_begin.y, glbPos_begin.z);
    points->InsertNextPoint(projection.x(), projection.y(), projection.z());

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
    actor->GetProperty()->SetColor(0.0, 0.0, 0.0);
    actor->GetProperty()->SetLineWidth(3);

    return actor;
}

// 绘制点到圆面的垂线
vtkSmartPointer<vtkActor> CDistance::pointToCircle()
{
    return 0;
}


int CLine::lineCount=0;
int CLine::currentLineId=0;
int CPoint::pointCount=0;
int CCircle::circleCount=0;
int CPlane::plainCount=0;
int CSphere::sphereCount=0;
int CCylinder::cylinderCount=0;
int CCone::coneCount=0;
int CDistance::currentCdistacneId=0;
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

QString CCircle::setentityinfo()
{
    QString infoText = QString("CenterX:%1\nCenterY:%2\nCenterZ:%3\n直径:%4").arg(m_pt.x).arg(m_pt.y).arg(m_pt.z).arg(m_d);
    return infoText;
}
CPosition CLine::getEnd() const
{
    return end;
}

void CLine::setEnd(const CPosition &newEnd)
{
    end = newEnd;
}

QString CLine::setentityinfo()
{
    QString infoText = QString("beginX:%1,beginY:%2,beginZ:%3\n endX:%4,endY:%5,endZ:%6").arg(begin.x).arg(begin.y).arg(begin.z)
                           .arg(end.x).arg(end.y).arg(end.z);
    return infoText;
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

QString CPlane::setentityinfo()
{
    QString infoText = QString("CenterX:%1\nCenterY:%2\nCenterZ:%3\n法向量:(%4,%5,%6)").arg(center.x).arg(center.y).arg(center.z).
                       arg(normal.x()).arg(normal.y()).arg(normal.z());
    return infoText;
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

double CDistance::getUptolerance()
{
    return uptolerance;
}

double CDistance::getUndertolerance()
{
    return undertolerance;
}

void CDistance::setUptolerance(double on)
{
    uptolerance=on;
}

void CDistance::setUndertolerance(double under)
{
    undertolerance=under;
}

void CDistance::setbegin(const CPosition &newbegin)
{
    begin=newbegin;
}

void CDistance::setend(const CPosition &newend)
{
    end=newend;
}

void CDistance::setplane(const CPlane &Plane)
{
    plane=Plane;
    isHavePlane = true;
}

void CDistance::setcircle(const CCircle &Circle)
{
    circle=Circle;
    isHaveCircle = true;
}

void CDistance::setline(const CLine &Line)
{
    line=Line;
    isHaveLine = true;
}

double CDistance::getdistancepoint()
{
    return sqrt(pow(begin.x - end.x, 2) + pow(begin.y - end.y, 2) + pow(begin.z - end.z, 2));
}

double CDistance::getdistanceplane()
{
    QVector4D normal = plane.getNormal();
    CPosition center = plane.getCenter();

    // 将法线单位化
    double norm_length = sqrt(normal.x() * normal.x() + normal.y() * normal.y() + normal.z() * normal.z());
    QVector4D unitNormal = normal / norm_length;

    // 计算点到平面的距离
    // 点到平面的距离公式: d = |(P - P0) · N| / ||N||
    double distance = fabs((begin.x - center.x) * unitNormal.x() +
                           (begin.y - center.y) * unitNormal.y() +
                           (begin.z - center.z) * unitNormal.z());

    return distance;
}

double CDistance::getdistancecircle()
{
    CPosition t=circle.getCenter();
    return sqrt(pow(begin.x - t.x, 2) + pow(begin.y - t.y, 2) + pow(begin.z - t.z, 2));
}

double CDistance::getdistanceline()
{
    CPosition P=begin;
    CPosition end=line.end;
    CPosition begin=line.begin;
    double ABx = end.x - begin.x;
    double ABy = end.y - begin.y;
    double ABz = end.z - begin.z;

    double APx = P.x - begin.x;
    double APy = P.y - begin.y;
    double APz = P.z - begin.z;

    // 计算 AB 的平方长度
    double AB_squared = ABx * ABx + ABy * ABy + ABz * ABz;
    if(AB_squared==0){
        return 0;
    }
    // 计算点 P 在 AB 上的投影比例 t
    double t = (ABx * APx + ABy * APy + ABz * APz) / AB_squared;

    // 判断 t 的值,待判断.........
    /*if (t < 0.0) {
        // 返回 P 到 A 的距离
        return sqrt(APx * APx + APy * APy + APz * APz);
    } else if (t > 1.0) {
        // 返回 P 到 B 的距离
        double BPx = P.x - end.x;
        double BPy = P.y - end.y;
        double BPz = P.z - end.z;
        return sqrt(BPx * BPx + BPy * BPy + BPz * BPz);
    }*/

    // 投影落在线段 AB 上
    CPosition projection = {
        begin.x + t * ABx,
        begin.y + t * ABy,
        begin.z + t * ABz
    };
    setProjection(projection);
    // 返回 P 到投影点的距离
    double projPx = P.x - projection.x;
    double projPy = P.y - projection.y;
    double projPz = P.z - projection.z;

    return sqrt(projPx * projPx + projPy * projPy + projPz * projPz);
}

double CDistance::getdistance()
{
    return distance;
}

void CDistance::setdistance(double d)
{
    distance=d;
}


bool CDistance::judge()
{
    if(distance<=uptolerance&&distance>=undertolerance){
        qualified=true;
    }
    return qualified;
}

void CDistance::setProjection(CPosition pos)
{
    Projection=pos;
}



