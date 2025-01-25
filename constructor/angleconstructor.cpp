#include "angleconstructor.h"

#include <QDebug>
#include <cmath>
#include<QVector4D>

CPosition subtract(const CPosition& a, const CPosition& b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

CPosition add(const CPosition& a, const CPosition& b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

CPosition multiply(const CPosition& a, double t) {
    return {a.x * t, a.y * t, a.z * t};
}

double dot(const CPosition& a, const CPosition& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

CPosition cross(const CPosition& a, const CPosition& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

double norm(const CPosition& a) {
    return std::sqrt(dot(a, a));
}

double angleBetween(const CPosition& a, const CPosition& b) {
    return std::acos(dot(a, b) / (norm(a) * norm(b)));
}


// 计算向量的点积
double dotProduct(const QVector4D& a, const QVector4D& b) {
    return QVector4D::dotProduct(a, b);
}

// 计算向量的叉积
static QVector4D crossProduct(const QVector4D& a, const QVector4D& b) {
    return QVector4D(
        a.y() * b.z() - a.z() * b.y(),
        a.z() * b.x() - a.x() * b.z(),
        a.x() * b.y() - a.y() * b.x(),
        0 // 对于三维向量，w 组件通常设为 0
        );
};

// 计算向量的范数
double norm(const QVector4D& a) {
    return a.length();
}

// 计算两个向量之间的夹角（弧度）
double angleBetween(const QVector4D& a, const QVector4D& b) {
    return std::acos(dotProduct(a, b) / (norm(a) * norm(b)));
}

// CLine与CPlane相关计算
namespace LinePlane {
// 计算直线与平面之间的夹角（弧度）
double angleBetween(const CLine& line, const CPlane& plane) {
    QVector4D lineDir = QVector4D(line.end.x - line.begin.x, line.end.y - line.begin.y, line.end.z - line.begin.z, 0);
    return angleBetween(lineDir, plane.getNormal());
}

// 计算直线在平面上的投影
CLine projectOntoPlane(const CLine& line, const CPlane& plane) {
    QVector4D lineDir = QVector4D(line.end.x - line.begin.x, line.end.y - line.begin.y, line.end.z - line.begin.z, 0);
    QVector4D projectionDir = lineDir - dotProduct(lineDir, plane.getNormal()) * plane.getNormal();

    CLine projection;
    projection.begin = line.begin;
    projection.end.x = line.begin.x + projectionDir.x();
    projection.end.y = line.begin.y + projectionDir.y();
    projection.end.z = line.begin.z + projectionDir.z();
    return projection;
}

// 计算直线延伸到平面上的交点
CPosition intersectionWithPlane(const CLine& line, const CPlane& plane) {
    QVector4D lineDir = QVector4D(line.end.x - line.begin.x, line.end.y - line.begin.y, line.end.z - line.begin.z, 0);
    QVector4D planeToPoint = QVector4D(line.begin.x - plane.getCenter().x, line.begin.y - plane.getCenter().y, line.begin.z - plane.getCenter().z, 0);
    double t = -dotProduct(planeToPoint, plane.getNormal()) / dotProduct(lineDir, plane.getNormal());

    CPosition intersection;
    intersection.x = line.begin.x + t * lineDir.x();
    intersection.y = line.begin.y + t * lineDir.y();
    intersection.z = line.begin.z + t * lineDir.z();
    return intersection;
}
}

// CPlane与CPlane相关计算
namespace PlanePlane {
// 计算两个平面之间的夹角（弧度）
double angleBetween(const CPlane& plane1, const CPlane& plane2) {
    return angleBetween(plane1.getNormal(), plane2.getNormal());
}

// 计算两个平面的交线
CLine intersectionLine(const CPlane& plane1, const CPlane& plane2) {
    QVector4D lineDir = crossProduct(plane1.getNormal(), plane2.getNormal());
    QVector4D pointOnLine = crossProduct(plane1.getNormal(), plane2.getNormal());

    CLine intersection;
    intersection.begin = { pointOnLine.x(), pointOnLine.y(), pointOnLine.z() };
    intersection.end = { pointOnLine.x() + lineDir.x(), pointOnLine.y() + lineDir.y(), pointOnLine.z() + lineDir.z() };
    return intersection;
}

// 计算垂直于两个平面的交线并且交点在交线上的线段
std::pair<CLine, CLine> perpendicularLines(const CPlane& plane1, const CPlane& plane2) {
    CLine intersection = intersectionLine(plane1, plane2);

    CLine line1;
    line1.begin = intersection.begin;
    line1.end = { intersection.begin.x + plane1.getNormal().x(), intersection.begin.y + plane1.getNormal().y(), intersection.begin.z + plane1.getNormal().z() };

    CLine line2;
    line2.begin = intersection.begin;
    line2.end = { intersection.begin.x + plane2.getNormal().x(), intersection.begin.y + plane2.getNormal().y(), intersection.begin.z + plane2.getNormal().z() };

    return { line1, line2 };
}

// 计算两条线段的交点
CPosition intersectionOfPerpendicularLines(const CLine& line1, const CLine& line2) {
    // 由于线段是垂直于交线的，因此它们的交点就是交线的开始点
    return line1.begin;
}
}

std::pair<CPosition, CPosition> closestPointsBetweenLines(const CLine& line1, const CLine& line2) {
    CPosition p1 = line1.getBegin(), q1 = line1.getEnd();
    CPosition p2 = line2.getBegin(), q2 = line2.getEnd();
    CPosition d1 = subtract(q1, p1), d2 = subtract(q2, p2);
    CPosition r = subtract(p1, p2);
    double a = dot(d1, d1);
    double e = dot(d2, d2);
    double f = dot(d2, r);

    double s, t;
    if (a <= 1e-10 && e <= 1e-10) {
        s = t = 0.0;
    } else if (a <= 1e-10) {
        s = 0.0;
        t = f / e;
    } else {
        double c = dot(d1, r);
        if (e <= 1e-10) {
            t = 0.0;
            s = -c / a;
        } else {
            double b = dot(d1, d2);
            double denom = a * e - b * b;
            if (denom != 0.0) {
                s = (b * f - c * e) / denom;
            } else {
                s = 0.0;
            }
            t = (b * s + f) / e;
        }
    }

    CPosition closestPointLine1 = add(p1, multiply(d1, s));
    CPosition closestPointLine2 = add(p2, multiply(d2, t));
    return {closestPointLine1, closestPointLine2};
}

bool areLinesSkew(const CLine& line1, const CLine& line2) {
    CPosition d1 = subtract(line1.getEnd(), line1.getBegin());
    CPosition d2 = subtract(line2.getEnd(), line2.getBegin());
    CPosition r = subtract(line1.getBegin(), line2.getBegin());
    return dot(cross(d1, d2), r) != 0;
}
CAngle* AngleConstructor::createAngle(CPlane* plane1,CPlane* plane2){
    double anglePlanePlane = PlanePlane::angleBetween(*plane1, *plane2);
    std::pair<CLine, CLine> perpendicularLines = PlanePlane::perpendicularLines(*plane1, *plane2);
    CPosition intersectionPerpendicularLines = PlanePlane::intersectionOfPerpendicularLines(perpendicularLines.first, perpendicularLines.second);
    return createAngle(&perpendicularLines.first,&perpendicularLines.second,anglePlanePlane,intersectionPerpendicularLines);

}
CAngle* AngleConstructor::createAngle(CLine* line1,CPlane* plane1){
    double angleLinePlane = LinePlane::angleBetween(*line1, *plane1);
    CLine projectedLine = LinePlane::projectOntoPlane(*line1, *plane1);
    CPosition intersectionLinePlane = LinePlane::intersectionWithPlane(*line1, *plane1);
    return createAngle(line1,&projectedLine,angleLinePlane,intersectionLinePlane);

}
CAngle* AngleConstructor::createAngle(CPoint*p1 ,CPoint* crossPosition,CPoint*p3)
{
    CLine line1,line2;
    CPosition P1=p1->GetPt(),P2=crossPosition->GetPt(),P3=p3->GetPt();
    line1.SetPosition(P1,P2);
    line2.SetPosition(P2,P3);
    return createAngle(&line1,&line2);
}
CAngle* AngleConstructor::createAngle(CLine* line1,CLine* line2,double angleValue){
    CAngle* angle=new CAngle();
    angle->setLine1(*line1);
    angle->setLine2(*line2);
    angle->setAngleValue(angleValue);
    return angle;
}
CAngle* AngleConstructor::createAngle(CLine* line1,CLine* line2,double angleValue,CPosition vertex){
    CAngle* angle=new CAngle();
    angle->setLine1(*line1);
    angle->setLine2(*line2);
    angle->setAngleValue(angleValue);
    angle->setVertex(vertex);
    return angle;
}
CEntity* AngleConstructor::create(QVector<CEntity*>& entitylist){
    QVector<CPoint*>points;
    QVector<CLine*>lines;
    QVector<CPlane*>planes;
    for(int i=0;i<entitylist.size();i++){
        CEntity* entity=entitylist[i];
        if(!entity->IsSelected())continue;
        if(entity->GetUniqueType()==enPoint){
            CPoint * point=(CPoint*)entity;
            points.push_back(point);
        }
        if(entity->GetUniqueType()==enLine){
            CLine* line=(CLine*)entity;
            lines.push_back(line);
        }
        if(entity->GetUniqueType()==enPlane){
            CPlane* plane=(CPlane*)entity;
            planes.push_back(plane);
        }
    }
    if(points.size()==3){
        CAngle * angle=createAngle(points[0],points[1],points[2]);
        angle->parent.append(points[0]);
        angle->parent.append(points[1]);
        angle->parent.append(points[2]);
        return angle ;
    }
    if(lines.size()==2){
         CAngle * angle=createAngle(lines[0],lines[1]);
        angle->parent.append(lines[0]);
         angle->parent.append(lines[1]);
        return  angle;
    }
    if(lines.size()==1&&planes.size()==1){
        CAngle * angle=createAngle(lines[0],planes[0]);
        angle->parent.append(lines[0]);
        angle->parent.append(planes[0]);
        return angle;
    }
    if(planes.size()==2){
        CAngle * angle=createAngle(planes[0],planes[1]);
        angle->parent.append(planes[0]);
        angle->parent.append(planes[1]);
        return angle;
    }
    if(points.size()<3&&lines.size()<2&&planes.size()<2){
         m_wrongInfo=SourceEntityLess;
    }
    else{
        m_wrongInfo=SourceEntityMuch;
    }

    return nullptr;
}
CAngle* AngleConstructor::createAngle(CLine* line1,CLine* line2){

    // 设置line1和line2的位置

    if (!areLinesSkew(*line1, *line2)) {
        // 共面
        CPosition d1 = subtract(line1->getEnd(), line1->getBegin());
        CPosition d2 = subtract(line2->getEnd(), line2->getBegin());
        CPosition r = subtract(line1->getBegin(), line2->getBegin());
        double t = dot(cross(r, d2), cross(d1, d2)) / norm(cross(d1, d2));
        CPosition intersection = add(line1->getBegin(), multiply(d1, t));
        double angle = angleBetween(d1, d2);

        return createAngle(line1,line2,angle,intersection);
    } else {
        // 异面
        auto [closestPointLine1, closestPointLine2] = closestPointsBetweenLines(*line1, *line2);
        CPosition midPoint = multiply(add(closestPointLine1, closestPointLine2), 0.5);
        double distance = norm(subtract(closestPointLine1, closestPointLine2));
        double angle = angleBetween(subtract(line1->getEnd(), line1->getBegin()), subtract(line2->getEnd(), line2->getBegin()));
        if (distance < 1e-3) {
            return createAngle(line1,line2,angle,midPoint);
        } else {
            return createAngle(line1,line2,angle);
        }


    }
}
