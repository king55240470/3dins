#include "centitytypes.h"
#include "mainwindow.h"

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
#include <vtkTransform.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkPolygon.h>
#include <vtkMath.h>
#include <vtkCubeSource.h>
#include <vtkDistancePolyDataFilter.h>

// 定义 getActorToPointCloud 和 actorToPointCloud;
QMap<vtkActor*, pcl::PointCloud<pcl::PointXYZRGB>> CPointCloud::actorToPointCloud;
QMap<vtkActor*, pcl::PointCloud<pcl::PointXYZRGB>> &CPointCloud::getActorToPointCloud(){
    return actorToPointCloud;
}

bool CPointCloud::haveSaved=false;
bool CPointCloud::haveOpened=false;
// int CPointCloud::pcCount=0;

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
    actor->GetProperty()->SetColor(MainWindow::ActorColor[0], MainWindow::ActorColor[1]
                                   ,MainWindow::ActorColor[2]);

    return actor;
}

QString CPoint::getCEntityInfo()
{
    QString infoText = QString("Information:\nX:%1\nY:%2\nZ:%3").arg(QString::number(m_pt.x, 'f',  3))
    .arg(QString::number(m_pt.y, 'f',  3)).arg(QString::number(m_pt.z, 'f',  3));
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
    actor->GetProperty()->SetColor(0, 0, 0);
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

    // 获取法向量并单位化
    QVector4D normalVec = getNormal();
    normalVec.normalize();

    // 创建圆上的点集
    auto points = vtkSmartPointer<vtkPoints>::New();
    const int numPoints = 100; // 圆的点数，更多点数会更平滑
    const double radius = getDiameter() / 2; // 圆的半径
    auto center = globalPos;

    // 创建一个与法向量正交的向量
    QVector3D v1(1, 0, 0);
    if (fabs(QVector3D::dotProduct(v1, normalVec.toVector3D())) > 0.99) {
        v1 = QVector3D(0, 1, 0);
    }
    QVector3D v2 = QVector3D::crossProduct(normalVec.toVector3D(), v1).normalized();
    v1 = QVector3D::crossProduct(v2, normalVec.toVector3D()).normalized();

    for (int i = 0; i < numPoints; ++i) {
        double theta = 2.0 * vtkMath::Pi() * static_cast<double>(i) / static_cast<double>(numPoints);
        double x = center.x + radius * (v1.x() * cos(theta) + v2.x() * sin(theta));
        double y = center.y + radius * (v1.y() * cos(theta) + v2.y() * sin(theta));
        double z = center.z + radius * (v1.z() * cos(theta) + v2.z() * sin(theta));
        points->InsertNextPoint(x, y, z);
    }

    // 创建一个线源来表示圆的线（多段线）
    auto lines = vtkSmartPointer<vtkCellArray>::New();
    vtkIdType pointIds[2];
    for (int i = 0; i < numPoints - 1; ++i) {
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

vtkSmartPointer<vtkActor> CPlane::draw() {
    // 获取图形在参考坐标系下的坐标，并计算全局坐标
    CPosition pos(getCenter().x, getCenter().y, getCenter().z);
    QVector4D posVec = GetRefCoord()->m_mat * QVector4D(pos.x, pos.y, pos.z, 1);
    CPosition globalPos(posVec.x(), posVec.y(), posVec.z());

    // 获取平面参数
    double halfL = getLength() /  2.0;
    double halfW = getWidth() /  2.0;

    // 获取法向量并单位化
    QVector4D normalVec = getNormal();
    double norm_length = sqrt(normalVec.x() * normalVec.x() +
                              normalVec.y() * normalVec.y() +
                              normalVec.z() * normalVec.z());
    QVector4D unitNormal = normalVec / norm_length;

    // 第一个向量是长边向量
    QVector3D firstPerpVec = dir_long_edge.toVector3D();

    // 验证 firstPerpVec 是否与 unitNormal 正交
    double dotProduct = QVector3D::dotProduct(firstPerpVec, unitNormal.toVector3D());
    if (fabs(dotProduct) > 1e-6) { // 如果不正交，重新计算正交向量
        firstPerpVec = QVector3D::crossProduct(unitNormal.toVector3D(), QVector3D(1, 0, 0));
        if (firstPerpVec.length() < 1e-6) { // 如果仍为零，改用另一个方向
            firstPerpVec = QVector3D::crossProduct(unitNormal.toVector3D(), QVector3D(0, 1, 0));
        }
    }

    // 将 firstPerpVec 单位化
    double norm_1 = sqrt(firstPerpVec.x() * firstPerpVec.x() +
                         firstPerpVec.y() * firstPerpVec.y() +
                         firstPerpVec.z() * firstPerpVec.z());
    QVector3D unitNormal_1 = firstPerpVec / norm_1;

    // 计算第二个正交向量
    QVector3D secondPerpVec = QVector3D::crossProduct(unitNormal_1, unitNormal.toVector3D());

    // 计算矩形四个顶点的全局坐标
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

    // 创建一个 vtkCellArray 对象来存储多边形
    vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
    vtkIdType verts[4] = {0, 1, 2, 3};
    cells->InsertNextCell(4, verts);

    // 设置多边形到 polyData
    polyData->SetPolys(cells);

    // 创建一个 vtkPolyDataMapper 来映射 polyData 到图形表示
    vtkSmartPointer<vtkPolyDataMapper> mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputData(polyData);

    // 创建一个 vtkActor 来表示多边形
    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(MainWindow::ActorColor[0], MainWindow::ActorColor[1]
                                   ,MainWindow::ActorColor[2]);

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
    actor->GetProperty()->SetColor(MainWindow::ActorColor[0], MainWindow::ActorColor[1]
                                   ,MainWindow::ActorColor[2]);

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

    // 创建变换对象，用于旋转圆柱方向
    vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
    // 默认轴向量为y轴
    QVector3D yAxis(0, 1, 0);
    // 归一化axis，作为目标轴向量
    QVector3D targetAxis(getAxis());
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

    // 创建映射器
    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(cylinder->GetOutputPort());

    // 创建执行器
    auto actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(MainWindow::ActorColor[0], MainWindow::ActorColor[1]
                                   ,MainWindow::ActorColor[2]);
    actor->SetUserTransform(transform); // 应用变换

    return actor;
}


QString CCone::getCEntityInfo()
{
    auto infoText = QString("Information:\ncenter: (%1,%2,%3)\nradian:%4\nheight:%5\naxis:(%6,%7,%8)")
    .arg(QString::number(getVertex().x, 'f',  3)).arg(QString::number(getVertex().y, 'f',  3)).arg(QString::number(getVertex().z, 'f',  3))
        .arg(QString::number(radian, 'f',  3)).arg(QString::number(height, 'f',  3))
        .arg(QString::number(axis.x(), 'f',  3)).arg(QString::number(axis.y(), 'f',  3)).arg(QString::number(axis.z(), 'f',  3));

    return infoText;
}

// 圆锥的draw()
vtkSmartPointer<vtkActor> CCone::draw(){
    // 获取图形在参考坐标系下的坐标(预置时输入的)，并计算得到他在机械坐标系下的位置(全局坐标)
    CPosition pos(getVertex().x, getVertex().y, getVertex().z);
    QVector4D posVec = GetRefCoord()->m_mat * QVector4D(pos.x, pos.y, pos.z, 1);
    QVector4D center=posVec-getAxis()*(getHeight()/2.0);
    CPosition globalPos(center.x(), center.y(), center.z());

    // 创建圆锥源
    auto cone = vtkSmartPointer<vtkConeSource>::New();
    cone->SetAngle(getRadian()*180/M_PI);
    cone->SetCenter(globalPos.x, globalPos.y, globalPos.z);
    cone->SetRadius(tan(getRadian()/2)*getHeight());
    cone->SetDirection(getAxis()[0], getAxis()[1], getAxis()[2]); // 设置轴向量
    cone->SetHeight(getHeight());
    cone->SetResolution(100);

    // 创建映射器
    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(cone->GetOutputPort());

    // 创建执行器
    auto actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(MainWindow::ActorColor[0], MainWindow::ActorColor[1]
                                   ,MainWindow::ActorColor[2]);

    return actor;
}

QString CCuboid::getCEntityInfo()
{
    auto infoText = QString("Center: (%1, %2, %3)\nLength: %4\nWidth: %5\nHeigth: %6\n"
                            "RotatedAngleX: %7\nRotatedAngleY: %8\nRotatedAngleZ: %9")
                        .arg(QString::number(getCenter().x, 'f',  3)).arg(QString::number(getCenter().y, 'f',  3))
                        .arg(QString::number(getCenter().z, 'f',  3))
                        .arg(QString::number(getLength(), 'f',  3)).arg(QString::number(getWidth(), 'f',  3))
                        .arg(QString::number(getHeight(), 'f',  3)).arg(QString::number(normal.x(), 'f',  3))
                        .arg(QString::number(normal.y(), 'f',  3)).arg(QString::number(normal.z(), 'f',  3));

    return infoText;
}

// 长方体的draw()
vtkSmartPointer<vtkActor> CCuboid::draw() {
    // 获取中心点
    CPosition pos(getCenter().x, getCenter().y, getCenter().z);
    QVector4D posVec = GetRefCoord()->m_mat * QVector4D(pos.x, pos.y, pos.z, 1);
    CPosition globalPos(posVec.x(), posVec.y(), posVec.z());

    // 创建长方体
    auto cuboid = vtkSmartPointer<vtkCubeSource>::New();
    cuboid->SetCenter(globalPos.x, globalPos.y, globalPos.z); // 设置长方体中心点
    cuboid->SetXLength(getLength());  // 设置长方体长度
    cuboid->SetYLength(getWidth());   // 设置长方体宽度
    cuboid->SetZLength(getHeight()); // 设置长方体高度

    // 获取法向量并归一化
    QVector4D normal = getNormal(); // 法向量 (QVector4D)
    QVector3D normalVec(normal.x(), normal.y(), normal.z());
    normalVec.normalize(); // 标准化法向量

    // 构造局部坐标系
    QVector3D up(0, 0, 1); // 默认参考向量，通常是 Z 轴方向
    if (qFuzzyCompare(normalVec, up)) {
        up = QVector3D(1, 0, 0); // 若法向量接近 Z 轴，改用 X 轴作为参考
    }
    QVector3D right = QVector3D::crossProduct(up, normalVec).normalized(); // 局部 X 轴
    QVector3D newUp = QVector3D::crossProduct(normalVec, right).normalized(); // 局部 Y 轴

    // 构造旋转矩阵
    vtkSmartPointer<vtkMatrix4x4> rotationMatrix = vtkSmartPointer<vtkMatrix4x4>::New();
    rotationMatrix->Identity();
    rotationMatrix->SetElement(0, 0, right.x());
    rotationMatrix->SetElement(1, 0, right.y());
    rotationMatrix->SetElement(2, 0, right.z());
    rotationMatrix->SetElement(0, 1, newUp.x());
    rotationMatrix->SetElement(1, 1, newUp.y());
    rotationMatrix->SetElement(2, 1, newUp.z());
    rotationMatrix->SetElement(0, 2, normalVec.x());
    rotationMatrix->SetElement(1, 2, normalVec.y());
    rotationMatrix->SetElement(2, 2, normalVec.z());

    // 创建变换对象并设置旋转矩阵
    vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
    transform->SetMatrix(rotationMatrix);

    // 创建映射器
    auto mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(cuboid->GetOutputPort());

    // 创建执行器
    auto actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);
    actor->GetProperty()->SetColor(0.5, 0.5, 0.5); // 设置颜色
    actor->SetUserTransform(transform); // 应用变换
    actor->GetProperty()->SetRepresentationToWireframe(); // 改为线框绘制
    actor->GetProperty()->SetLineWidth(2);

    return actor;
}

int CPointCloud::pointCloudCount = 0;
QString CPointCloud::getCEntityInfo()
{
    auto infoText = QString ("PointCloud\nNumbers of points: %1")
    .arg(QString::number(getPointCloudSize(), 'f', 0));

    return infoText;
}

vtkSmartPointer<vtkActor> CPointCloud::draw(){
    // 将cloud转换为VTK的点集
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
    colors->SetNumberOfComponents(3);
    colors->SetNumberOfTuples(m_pointCloud.points.size());
    colors->SetName("Colors");

    points->SetNumberOfPoints(m_pointCloud.points.size());
    for (size_t i = 0; i < m_pointCloud.points.size(); ++i)
    {
        points->SetPoint(i, m_pointCloud.points[i].x, m_pointCloud.points[i].y, m_pointCloud.points[i].z);
        // 如果是对比生成的点云则设置颜色
        if(isComparsionCloud)
            colors->SetTuple3(i, m_pointCloud.points[i].r, m_pointCloud.points[i].g, m_pointCloud.points[i].b);
        else{
            m_pointCloud.points[i].r = MainWindow::ActorColor[0] * 255;
            m_pointCloud.points[i].g = MainWindow::ActorColor[1] * 255;
            m_pointCloud.points[i].b = MainWindow::ActorColor[2] * 255;
            colors->SetTuple3(i, m_pointCloud.points[i].r, m_pointCloud.points[i].g, m_pointCloud.points[i].b);
        }
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
    if(isFileCloud){ // 如果是文件点云则统一设置成灰色
        actor->GetProperty()->SetColor(MainWindow::ActorColor[0], MainWindow::ActorColor[1]
                                       ,MainWindow::ActorColor[2]);
    }

    return actor;
}

// 各种距离的draw
vtkSmartPointer<vtkActor> CDistance::draw(){
    vtkSmartPointer<vtkActor> actor;

    if(isPointToPlane)
        actor = pointToPlane();
    else if(isPointToLine)
        actor = pointToLine();
    else if(isPointToPoint){
        actor = pointToPoint();
    }
    else if(isPlaneToPlane){
        actor = planeToPlane();
    }

    return actor;
}

// 绘制点到面的垂线
vtkSmartPointer<vtkActor> CDistance::pointToPlane()
{
    // 取平面法向量并单位化
    QVector4D plane_normal = plane.getNormal();
    plane_normal.normalize();

    // 将begin转为全局坐标
    CPosition pos_begin(begin.x, begin.y, begin.z);
    QVector4D posVec_begin = GetRefCoord()->m_mat * QVector4D(pos_begin.x, pos_begin.y, pos_begin.z, 1);
    CPosition glbPos_begin(posVec_begin.x(), posVec_begin.y(), posVec_begin.z());

    // 计算点到平面的距离
    double distance = getdistanceplane();

    // 计算glPos_begin在平面上的落点
    CPosition projection;
    if (fabs(plane_normal.z()) > 1e-6) { // 检查z分量是否足够大，以避免除以0
        projection.x = glbPos_begin.x - distance * plane_normal.x();
        projection.y = glbPos_begin.y - distance * plane_normal.y();
        projection.z = glbPos_begin.z - distance * plane_normal.z();
    } else {
        // 如果z分量接近0，则假设平面在xy平面上，直接使用xy坐标
        projection.x = glbPos_begin.x - distance * plane_normal.x();
        projection.y = glbPos_begin.y - distance * plane_normal.y();
        projection.z = glbPos_begin.z; // 使用平面的z坐标
    }

    // 创建点集，并插入定义线的两个点
    auto points = vtkSmartPointer<vtkPoints>::New();
    points->InsertNextPoint(glbPos_begin.x, glbPos_begin.y, glbPos_begin.z);
    points->InsertNextPoint(projection.x, projection.y, projection.z);

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
    actor->GetProperty()->SetColor(MainWindow::ActorColor);
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
    CPosition line_begin = line.getBegin();
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
    // p = p0 + v.w.w / |w|^2
    QVector3D projection = QVector3D(lineVec_begin) + dotProduct * lineVec;
    //Projection.x=projection.x();
    //Projection.y=projection.y();
    //Projection.z=projection.z();

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
    actor->GetProperty()->SetColor(MainWindow::ActorColor);
    actor->GetProperty()->SetLineWidth(3);

    return actor;
}

// 绘制点到圆面的垂线
vtkSmartPointer<vtkActor> CDistance::pointToCircle()
{
    return 0;
}

// 绘制点到点的距离
vtkSmartPointer<vtkActor> CDistance::pointToPoint()
{
    // 将begin、end转为全局坐标
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
    actor->GetProperty()->SetColor(MainWindow::ActorColor);
    actor->GetProperty()->SetLineWidth(3);
    return actor;
}

vtkSmartPointer<vtkActor> CDistance::planeToPlane()
{
    return nullptr;
}

//角度绘制
vtkSmartPointer<vtkActor> CAngle::draw() {
    // 获取顶点和两条线的端点
    CPosition pos_vertex(vertex.x, vertex.y, vertex.z);
    QVector4D posVec_vertex = GetRefCoord()->m_mat * QVector4D(pos_vertex.x, pos_vertex.y, pos_vertex.z, 1);
    CPosition globalPos_vertex(posVec_vertex.x(), posVec_vertex.y(), posVec_vertex.z());

    CPosition pos_line1_end(line1.getEnd().x, line1.getEnd().y, line1.getEnd().z);
    QVector4D posVec_line1_end = GetRefCoord()->m_mat * QVector4D(pos_line1_end.x, pos_line1_end.y, pos_line1_end.z, 1);
    CPosition globalPos_line1_end(posVec_line1_end.x(), posVec_line1_end.y(), posVec_line1_end.z());

    CPosition pos_line2_end(line2.getEnd().x, line2.getEnd().y, line2.getEnd().z);
    QVector4D posVec_line2_end = GetRefCoord()->m_mat * QVector4D(pos_line2_end.x, pos_line2_end.y, pos_line2_end.z, 1);
    CPosition globalPos_line2_end(posVec_line2_end.x(), posVec_line2_end.y(), posVec_line2_end.z());

    // 创建点集，并插入定义角度的三个点
    auto points = vtkSmartPointer<vtkPoints>::New();
    points->InsertNextPoint(globalPos_vertex.x, globalPos_vertex.y, globalPos_vertex.z);
    points->InsertNextPoint(globalPos_line1_end.x, globalPos_line1_end.y, globalPos_line1_end.z);
    points->InsertNextPoint(globalPos_line2_end.x, globalPos_line2_end.y, globalPos_line2_end.z);

    // 创建线源
    auto lines = vtkSmartPointer<vtkCellArray>::New();
    vtkIdType line1[2] = {0, 1}; // 顶点到线1端点
    vtkIdType line2[2] = {0, 2}; // 顶点到线2端点
    lines->InsertNextCell(2, line1);
    lines->InsertNextCell(2, line2);

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
    actor->GetProperty()->SetColor(MainWindow::ActorColor);
    actor->GetProperty()->SetLineWidth(3);

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
int CDistance::currentCdistacneId=0;
int CCuboid::cuboidCount=0;
int CAngle::currentCAngleId=0;

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

QVector4D CCircle::getNormal() const
{
    return normal;
}

void CCircle::setNormal(const QVector4D &newNormal)
{
    normal = newNormal;
}

int CCircle::getId()
{
    return currentCircleId;
}

QString CCircle::getCEntityInfo()
{
    QString infoText = QString("Information:\nCenterX: %1\nCenterY: %2\nCenterZ: %3\ndiameter: %4")
    .arg(QString::number(m_pt.x, 'f',  3)).arg(QString::number(m_pt.y, 'f',  3)).arg(QString::number(m_pt.z, 'f',  3)).arg(QString::number(m_d, 'f',  3));
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

QString CLine::getCEntityInfo()
{
    QString infoText = QString("Information:\nbeginX: %1,beginY: %2,beginZ: %3\n endX: %4,endY: %5,endZ: %6").arg(QString::number(begin.x, 'f',  3))
    .arg(QString::number(begin.y, 'f',  3)).arg(QString::number(begin.z, 'f',  3))
        .arg(QString::number(end.x, 'f',  3)).arg(QString::number(end.y, 'f',  3)).arg(QString::number(end.z, 'f',  3));
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

QString CPlane::getCEntityInfo()
{
    QString infoText = QString("Information:\nCenterX: %1\nCenterY: %2\nCenterZ: %3\nnormal:(%4,%5,%6)\nedge:(%7,%8,%9)\nlength,width:(%10,%11)").arg(QString::number(center.x, 'f',  3)).arg(QString::number(center.y, 'f',  3)).arg(QString::number(center.z, 'f',  3)).
                       arg(QString::number(normal.x(), 'f',  3)).arg(GetObjectCName())
                           .arg(QString::number(normal.y(), 'f',  3)).arg(QString::number(normal.z(), 'f',  3))
                           .arg(QString::number(dir_long_edge.x(), 'f',  3)).arg(QString::number(dir_long_edge.y(), 'f',  3))
                           .arg(QString::number(dir_long_edge.z(), 'f',  3)).arg(QString::number(length, 'f',  3)).arg(QString::number(width, 'f',  3));
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

QString CSphere::getCEntityInfo()
{
    QString infoText = QString("Information:\nCenterX: %1\nCenterY: %2\nCenterZ: %3\ndiameter: %4")
    .arg(QString::number(center.x, 'f',  3)).arg(QString::number(center.y, 'f',  3))
        .arg(QString::number(center.z, 'f',  3)).arg(QString::number(diameter, 'f',  3));
    return infoText;
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

QString CCylinder::getCEntityInfo()
{
    QString infoText = QString("Information:\nX: %1\nY: %2\nZ: %3\ndiameter: %4\nheight: %5\naxial:(%6,%7,%8)")
    .arg(QString::number(btm_center.x, 'f',  3)).arg(QString::number(btm_center.y, 'f',  3))
        .arg(QString::number(btm_center.z, 'f',  3)).arg(QString::number(diameter, 'f',  3))
        .arg(QString::number(height, 'f',  3)).arg(QString::number(axis.x(), 'f',  3))
        .arg(QString::number(axis.y(), 'f',  3)).arg(QString::number(axis.z(), 'f',  3));
    return infoText;
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

CPosition CCuboid::getCenter() const
{
    return center;
}

void CCuboid::setCenter(const CPosition &newCenter)
{
    center=newCenter;
}

double CCuboid::getLength() const
{
    return length;
}

void CCuboid::setLength(double newLength)
{
    length=newLength;
}

double CCuboid::getWidth() const
{
    return width;
}

void CCuboid::setWidth(double newWidth)
{
    width=newWidth;
}

double CCuboid::getHeight() const
{
    return height;
}

void CCuboid::setHeight(double newHeight)
{
    height=newHeight;
}

QVector4D CCuboid::getNormal() const
{
    return normal;
}

void CCuboid::setNormal(const QVector4D &newNormal)
{
    normal = newNormal;
}


QString CDistance::getCEntityInfo()
{
    QString type_str;
    QString upTol_str;
    QString underTol_str;
    QString q;
    // 判断是哪种距离
    if(isPointToPlane)
        type_str = QString("pointToPlane distance: %1\n").arg(QString::number(getdistanceplane(), 'f',  3));
    else if(isPointToLine)
        type_str = QString("pointToLine distance: %1\n").arg(QString::number(getdistanceline(), 'f',  3));
    else if(isPointToCircle){
        type_str = QString("pointToCircle distance: %1\n").arg(QString::number(getdistancecircle(), 'f',  3));
    }
    else if(isPointToPoint){
        type_str = QString("pointToPoint distance: %1\n").arg(QString::number(getdistancepoint(), 'f',  3));
    }
    upTol_str = QString("upTolerance: %1\n").arg(QString::number(getUptolerance(), 'f',  3));
    underTol_str = QString("underTolerance: %1\n").arg(QString::number(getUndertolerance(), 'f',  3));
    q=QString("quality: %1、n").arg(judge());
    return type_str + upTol_str + underTol_str+q;
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
    isPointToPoint = true;
}

void CDistance::setplane(const CPlane &Plane)
{
    plane=Plane;
    isPointToPlane = true;
}

void CDistance::setcircle(const CCircle &Circle)
{
    circle=Circle;
    isPointToCircle = true;
}

void CDistance::setline(const CLine &Line)
{
    line=Line;
    isPointToLine = true;
}

CPosition CDistance::getbegin()
{
    return begin;
}

CPosition CDistance::getProjection()
{
    return Projection;
}

double CDistance::getdistancepoint()
{
    return sqrt(pow(begin.x - end.x,  2) + pow(begin.y - end.y,  2) + pow(begin.z - end.z,  2));
}

double CDistance::getdistanceplane()
{
    // 将begin转为全局坐标
    CPosition pos_begin(begin.x, begin.y, begin.z);
    QVector4D posVec_begin = GetRefCoord()->m_mat * QVector4D(pos_begin.x, pos_begin.y, pos_begin.z, 1);
    CPosition glbPos_begin(posVec_begin.x(), posVec_begin.y(), posVec_begin.z());

    // 取平面中心并转为全局坐标
    CPosition plane_point = plane.getCenter();
    QVector4D posVec_center = GetRefCoord()->m_mat * QVector4D(plane_point.x, plane_point.y, plane_point.z,1);
    CPosition glbPos_center(posVec_center.x(), posVec_center.y(), posVec_center.z());

    // 将法线单位化
    QVector4D normal = plane.getNormal();
    double norm_length = sqrt(normal.x() * normal.x() + normal.y() * normal.y() + normal.z() * normal.z());
    QVector4D unitNormal = normal / norm_length;

    // 计算点到平面的距离
    // 点到平面的距离公式: d = |(P - P0) · N| / ||N||
    QVector3D direction = QVector3D(glbPos_begin.x - glbPos_center.x,
                                    glbPos_begin.y - glbPos_center.y,
                                    glbPos_begin.z - glbPos_center.z);
    // 使用点积自动判定begin与法向量正向还是反向
    double distance = QVector3D::dotProduct(direction, unitNormal.toVector3D()) / unitNormal.length();
    return distance;
}

double CDistance::getdistancecircle()
{
    CPosition t=circle.getCenter();
    return sqrt(pow(begin.x - t.x,  3) + pow(begin.y - t.y,  3) + pow(begin.z - t.z,  3));
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
    if(isPointToPoint){
        return getdistancepoint();
    }else if(isPointToLine){
        return getdistanceline();
    }else if(isPointToCircle){
        return getdistancecircle();
    }else if(isPointToPlane){
        return getdistanceplane();
    }
    return 0;
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

double CAngle::getAngleValue() const {
    return angleValue;
}

void CAngle::setAngleValue(double value) {
    angleValue = value;
}

double CAngle::getUptolerance() const {
    return uptolerance;
}

void CAngle::setUptolerance(double value) {
    uptolerance = value;
}

double CAngle::getUndertolerance() const {
    return undertolerance;
}

void CAngle::setUndertolerance(double value) {
    undertolerance = value;
}

CPosition CAngle::getVertex() const {
    return vertex;
}

void CAngle::setVertex(const CPosition &value) {
    vertex = value;
}

CLine CAngle::getLine1() const {
    return line1;
}

void CAngle::setLine1(const CLine &value) {
    line1 = value;
}

CLine CAngle::getLine2() const {
    return line2;
}

void CAngle::setLine2(const CLine &value) {
    line2 = value;
}

bool CAngle::isQualified() const {
    return qualified;
}

void CAngle::setQualified(bool value) {
    qualified = value;
}

bool CAngle::judge()
{
    if(angleValue<=uptolerance&&angleValue>=undertolerance){
        qualified=true;
    }
    return qualified;
}

QString CAngle::getCEntityInfo() {
    QString infoText = QString("Angle Information:\nVertex: (%1, %2, %3)\nLine1 End: (%4, %5, %6)\nLine2 End: (%7, %8, %9)\nAngle Value: %10\nUp Tolerance: %11\nUnder Tolerance: %12\nQualified: %13")
        .arg(QString::number(vertex.x, 'f', 3)).arg(QString::number(vertex.y, 'f', 3)).arg(QString::number(vertex.z, 'f', 3))
        .arg(QString::number(line1.getEnd().x, 'f', 3)).arg(QString::number(line1.getEnd().y, 'f', 3)).arg(QString::number(line1.getEnd().z, 'f', 3))
        .arg(QString::number(line2.getEnd().x, 'f', 3)).arg(QString::number(line2.getEnd().y, 'f', 3)).arg(QString::number(line2.getEnd().z, 'f', 3))
        .arg(QString::number(angleValue, 'f', 3))
        .arg(QString::number(uptolerance, 'f', 3))
        .arg(QString::number(undertolerance, 'f', 3))
        .arg(qualified ? "Yes" : "No");
    return infoText;
}




