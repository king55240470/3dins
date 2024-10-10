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
static auto isRightAngle=[](const QVector4D& a, const QVector4D& b) {
    return qFabs(QVector4D::dotProduct(a, b)) < 1e-5; // 使用小值判断近似直角
};
static auto isRectangle=[](const QVector4D& p1, const QVector4D& p2, const QVector4D& p3, const QVector4D& p4) {
    QVector4D ab = p2 - p1;
    QVector4D ac = p3 - p1;
    QVector4D ad = p4 - p1;

    QVector4D bc = p3 - p2;
    QVector4D bd = p4 - p2;

    QVector4D cd = p4 - p3;

    // 检查所有角是否为直角
    return isRightAngle(ab, ac) && isRightAngle(ab, ad) &&
           isRightAngle(bc, bd) && isRightAngle(cd, ac) &&
           qFabs(ab.lengthSquared() - ac.lengthSquared()) < 1e-5;
};
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
bool RectangleConstructor::setRectangle(QVector<CEntity*>& entitylist){
    QVector<CPoint*>points;
    for(int i=0;i<entitylist.size();i++){
        CEntity* entity=entitylist[i];
        if(!entity->IsSelected()||entity->GetUniqueType()!=enPoint){
            continue;
        }
        CPoint * point=(CPoint*)entity;
        points.push_back(point);
    }
    if(points.size()==3){
        return setRectangle(*points[0],*points[1],*points[2]);
    }else if(points.size()==4){
        return setRectangle(*points[0],*points[1],*points[2],*points[3]);
    }
    return false;
}
bool RectangleConstructor::setRectangle(CPosition p1,CPosition p2,CPosition p3){
    PlaneConstructor pc;
    bool ok=pc.setPlane(p1,p2,p3);
    if(ok==false){
        return false;
    }
    m_rectangle=pc.getPlane();
    return true;
}
bool RectangleConstructor::setRectangle(CPoint p1,CPoint p2,CPoint p3){
    CPosition pt1=p1.GetPt();
    CPosition pt2=p2.GetPt();
    CPosition pt3=p3.GetPt();
    return setRectangle(pt1,pt2,pt3);
}
bool RectangleConstructor::setRectangle(CPosition pt1,CPosition pt2,CPosition pt3,CPosition pt4){
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
        m_rectangle.setCenter(center);
        m_rectangle.setNormal(ans.normal);
        m_rectangle.setDir_long_edge(ans.direction);
        m_rectangle.setLength(ans.length);
        m_rectangle.setWidth(ans.width);
        return true;
    }else{
        return false;
    }

}
bool RectangleConstructor::setRectangle(CPoint p1,CPoint p2,CPoint p3,CPoint p4){
    CPosition pt1=p1.GetPt();
    CPosition pt2=p2.GetPt();
    CPosition pt3=p3.GetPt();
    CPosition pt4=p4.GetPt();
    return setRectangle(pt1,pt2,pt3,pt4);
}
CPlane RectangleConstructor::getRectabgle(){
    return m_rectangle;
}
