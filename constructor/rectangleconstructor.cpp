#include "rectangleconstructor.h"
static QVector4D crossProduct(const QVector4D& a, const QVector4D& b) {
    return QVector4D(
        a.y() * b.z() - a.z() * b.y(),
        a.z() * b.x() - a.x() * b.z(),
        a.x() * b.y() - a.y() * b.x(),
        0 // 对于三维向量，w 组件通常设为 0
        );
};
struct RectangleInfo {
    QVector4D center;
    QVector4D normal;
    QVector4D direction;
    double length;
    double width;
    RectangleInfo(QVector4D center,QVector4D normal,QVector4D direction,double length,double width){
        this->center=center;
        this->normal=normal;
        this->direction=direction;
        this->length=length;
        this->width=width;
    }
    RectangleInfo(){}
};
//四个点首先判断是否形成矩形
bool isRightAngle(const QVector4D& a, const QVector4D& b) {
    return qFabs(QVector4D::dotProduct(a, b)) < 1e-5;
}

bool isRectangle(const QVector4D& p1, const QVector4D& p2, const QVector4D& p3, const QVector4D& p4) {
    QVector4D ab = p2 - p1;
    QVector4D bc = p3 - p2;
    QVector4D cd = p4 - p3;
    QVector4D da = p1 - p4;

    QVector4D diag1 = p3 - p1;
    QVector4D diag2 = p4 - p2;

    // 检查对边是否相等
    bool sidesEqual = qFabs(ab.lengthSquared() - cd.lengthSquared()) < 1e-5 &&
                      qFabs(bc.lengthSquared() - da.lengthSquared()) < 1e-5;

    // 检查对角线是否相等
    bool diagonalsEqual = qFabs(diag1.lengthSquared() - diag2.lengthSquared()) < 1e-5;

    // 检查所有角是否为直角
    bool anglesRight = isRightAngle(ab, bc) && isRightAngle(bc, cd) &&
                       isRightAngle(cd, da) && isRightAngle(da, ab);

    return sidesEqual && diagonalsEqual && anglesRight;
}
//根据顺序的四个点计算矩形信息
static auto calculateRectangleInfo=[](const QVector4D& p1, const QVector4D& p2, const QVector4D& p3, const QVector4D& p4) {
    QVector4D lengths[4] = {
        p2 - p1,
        p3 - p2,
        p4 - p3,
        p1 - p4
    };

    double length1 = lengths[0].length();
    double width1 = lengths[1].length();

    QVector4D center = (p1 + p2 + p3 + p4) / 4.0;

    QVector4D normal = crossProduct(lengths[0], lengths[1]).normalized();

    QVector4D direction = lengths[0].normalized();

    return RectangleInfo(center,normal,direction,length1,width1);
};


//检查四个点的所有顺序组合，找到满足条件的一组
static auto checkAllCombinations=[](const QVector<QVector4D>& points,RectangleInfo& ans) {
    for (int i = 0; i < points.size(); ++i) {
        for (int j = 0; j < points.size(); ++j) {
            if (j == i) continue;
            for (int k = 0; k < points.size(); ++k) {
                if (k == i || k == j) continue;
                for (int l = 0; l < points.size(); ++l) {
                    if (l == i || l == j || l == k) continue;

                    if (isRectangle(points[i], points[j], points[k], points[l])) {
                        ans=calculateRectangleInfo(points[i], points[j], points[k], points[l]);
                        return true;
                    }
                }
            }
        }
    }
    return false;
};
RectangleConstructor::RectangleConstructor() {}
CEntity* RectangleConstructor::create(QVector<CEntity*>& entitylist){
    Constructor::create(entitylist);
    QVector<CPoint*>point;
    for(int i=0;i<entitylist.size();i++){
        CEntity* entity=entitylist[i];
        if(!entity->IsSelected())continue;
        if(entity->GetUniqueType()==enPoint){
            CPoint * point1=(CPoint*)entity;
            point.push_back(point1);
        }
    }
    QVector<CPosition>&positions=Constructor::getPositions();//存储有效点
    if(point.size()==3&&positions.size()==3){
        CPlane*rectangle=createRectangle(positions[0],positions[1],positions[2]);
        rectangle->setCurrentId();
        rectangle->parent.push_back(point[0]);
        rectangle->parent.push_back(point[1]);
        rectangle->parent.push_back(point[2]);
        return rectangle;
    }else if(point.size()==4&&positions.size()==4){
        CPlane*rectangle=createRectangle(positions[0],positions[1],positions[2],positions[3]);
        rectangle->setCurrentId();
        rectangle->parent.push_back(point[0]);
        rectangle->parent.push_back(point[1]);
        rectangle->parent.push_back(point[2]);
        rectangle->parent.push_back(point[3]);
        return rectangle;
    }else if(positions.size()>4){
        setWrongInformation(PointTooMuch);
    }else if(positions.size()<3){
        setWrongInformation(PointTooLess);
        qDebug()<<"构造矩形点的数目"<<positions.size();
    }
    return nullptr;
}
CPlane* RectangleConstructor::createRectangle(CPosition p1,CPosition p2,CPosition p3){
    PlaneConstructor pc;
    return pc.createPlane(p1,p2,p3);
}

CPlane* RectangleConstructor::createRectangle(CPosition pt1,CPosition pt2,CPosition pt3,CPosition pt4){
    CPosition A=pt1;
    CPosition B=pt2;
    CPosition C=pt3;
    CPosition D=pt4;
    QVector<QVector4D> points;
    points.push_back(QVector4D(A.x, A.y, A.z, 1.0));
    points.push_back(QVector4D(B.x, B.y, B.z, 1.0));
    points.push_back(QVector4D(C.x, C.y, C.z, 1.0));
    points.push_back(QVector4D(D.x, D.y, D.z, 1.0));
    RectangleInfo ans;
    if( checkAllCombinations(points,ans)){
        CPosition center(ans.center.x(),ans.center.y(),ans.center.z());
        return createRectangle(center,ans.normal,ans.direction,ans.length,ans.width);
    }else{
        setWrongInformation(PointDontMatch);
        return nullptr;
    }

}
CPlane* RectangleConstructor::createRectangle(CPosition posCenter, QVector4D normal, QVector4D direction, double length, double width){
    PlaneConstructor constructor;
    CPlane* newplane=nullptr;
    newplane= constructor.createPlane(posCenter,normal,direction,length,width);
    setWrongInformation(constructor.getWrongInformation());
    return newplane;
}
