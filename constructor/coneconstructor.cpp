#include "coneconstructor.h"

ConeConstructor::ConeConstructor() {}
CEntity* ConeConstructor::create(QVector<CEntity*>& entitylist){
    Constructor::create(entitylist);
    QVector<CPoint*>points;
    for(int i=0;i<entitylist.size();i++){
        CEntity* entity=entitylist[i];
        if(!entity->IsSelected())continue;
        if(entity->GetUniqueType()==enPoint){
            CPoint * point=(CPoint*)entity;
            points.push_back(point);
        }
    }
    QVector<CPosition>& positions=Constructor::getPositions();
    if(positions.size()==3){
        CCone*cone=createCone(positions[0],positions[1],positions[2]);
        cone->parent.push_back(points[0]);
        cone->parent.push_back(points[1]);
        cone->parent.push_back(points[2]);
        return cone;
    }
    if(points.size()<3){
        setWrongInformation(PointTooLess);
    }else if(points.size()>3){
        setWrongInformation(PointTooMuch);
    }
    return nullptr;
}
CCone* ConeConstructor::createCone(CPosition p1,CPosition p2,CPosition p3){
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
    // 计算部分高度（可自定义，假设为完整高度的一半）
    double partH = fullH; // 结果是 4

    // 计算角度
    double radius = (bottomPoint - QVector4D(posCenter.x, posCenter.y, posCenter.z,0)).length(); // 结果是 5
    double angle = qRadiansToDegrees(atan(radius / fullH)); // 计算夹角
    // 调用函数
    return createCone(posCenter,axisVector,partH,fullH,angle);
}
CCone* ConeConstructor::createCone(CPosition posCenter, QVector4D axis, double partH, double fullH, double angle){
    CCone* newCone=new CCone();
    newCone->setCone_height(partH);
    newCone->setHeight(fullH);
    newCone->setAxis(axis);
    newCone->setRadian(angle);
    newCone->setVertex(posCenter);
    return newCone;

}
