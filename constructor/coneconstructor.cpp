#include "coneconstructor.h"

ConeConstructor::ConeConstructor() {}
bool ConeConstructor::setCone(QVector<CEntity*>& entitylist){
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
        return setCone(*points[0],*points[1],*points[2]);
    }
    return false;
}
bool ConeConstructor::setCone(CPosition p1,CPosition p2,CPosition p3){
    CPosition A=p1;
    CPosition B=p2;
    CPosition C=p3;

    QVector4D vecA(A.x, A.y, A.z, 1.0);
    QVector4D vecB(B.x, B.y, B.z, 1.0);
    QVector4D vecC(C.x, C.y, C.z, 1.0);

    CPosition posCenter(A); // 底面中心
    QVector4D bottomPoint(B.x,B.y , B.z,0); // 底面圆上的一点
    QVector4D topVertex(C.x, C.y, C.z,0);  // 圆锥顶点


    // 计算轴向
    QVector4D axis = (topVertex - QVector4D(posCenter.x, posCenter.y, posCenter.z,0));
    QVector4D axisVector(axis.x(), axis.y(), axis.z(), 0);  // 添加 0 作为第四个分量

    // 计算完整高度
    double fullH = axis.length();  // 结果是 8
    qDebug()<<fullH;
    // 计算部分高度（可自定义，假设为完整高度的一半）
    double partH = fullH; // 结果是 4

    // 计算角度
    double radius = (bottomPoint - QVector4D(posCenter.x, posCenter.y, posCenter.z,0)).length(); // 结果是 5
    double angle = qRadiansToDegrees(atan(radius / fullH)); // 计算夹角
    qDebug()<<radius<<"    "<<angle;
    qDebug()<<toQVector4D(posCenter);qDebug()<<bottomPoint<<topVertex;
    // 调用函数
   m_cone.setCone_height(partH);
   m_cone.setHeight(fullH);
   m_cone.setAxis(axisVector);
   m_cone.setRadian(angle);
   m_cone.setVertex(posCenter);
   return true;
}
bool ConeConstructor::setCone(CPoint p1,CPoint p2,CPoint p3){
    CPosition pt1=p1.GetPt();
    CPosition pt2=p2.GetPt();
    CPosition pt3=p3.GetPt();

    return setCone(pt1,pt2,pt3);
}
CCone ConeConstructor::getCone(){
    return m_cone;
}
