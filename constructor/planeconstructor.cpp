#include "planeconstructor.h"
#include"cmath"
#include <QMessageBox>
static QVector4D crossProduct(const QVector4D& a, const QVector4D& b) {
    return QVector4D(
        a.y() * b.z() - a.z() * b.y(),
        a.z() * b.x() - a.x() * b.z(),
        a.x() * b.y() - a.y() * b.x(),
        0 // 对于三维向量，w 组件通常设为 0
        );
};
float dotProduct(const QVector4D& v1, const QVector4D& v2) {
    return v1.x() * v2.x() + v1.y() * v2.y() + v1.z() * v2.z();
}
float length(const QVector4D& v) {
    return std::sqrt(v.x() * v.x() + v.y() * v.y() + v.z() * v.z());
}

// 判断哪个点的角度近似为90度
float angleBetweenVectors(const QVector4D& v1, const QVector4D& v2) {
    float dot = dotProduct(v1, v2);
    float len1 = length(v1);
    float len2 = length(v2);
    return std::acos(dot / (len1 * len2));
}

// 判断哪个点的角度在75度到105度之间
int findRightAngleVertex(const QVector4D& p1, const QVector4D& p2, const QVector4D& p3) {
    QVector4D edge1 = p2 - p1;
    QVector4D edge2 = p3 - p2;
    QVector4D edge3 = p1 - p3;

    float angle1 = angleBetweenVectors(edge1, -edge3);
    float angle2 = angleBetweenVectors(edge1, -edge2);
    float angle3 = angleBetweenVectors(edge2, -edge3);

    // 将角度从弧度转换为度数
    angle1 = angle1 * 180.0f / M_PI;
    angle2 = angle2 * 180.0f / M_PI;
    angle3 = angle3 * 180.0f / M_PI;

    // 判断哪个点的角度在75度到105度之间
    if (angle1 >= 75.0f && angle1 <= 105.0f) {
        return 1; // 点p1的角度近似为直角
    } else if (angle2 >= 75.0f && angle2 <= 105.0f) {
        return 2; // 点p2的角度近似为直角
    } else if (angle3 >= 75.0f && angle3 <= 105.0f) {
        return 3; // 点p3的角度近似为直角
    } else {
        return 0; // 没有点的角度近似为直角
    }
}
static auto distance(const CPosition& p1, const CPosition& p2) {
    return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2) + pow(p2.z - p1.z, 2));
};
static double distance(QVector4D& Edge){
    return sqrt(pow(Edge.x(),2)+pow(Edge.y(),2)+pow(Edge.z(),2));
}
void InformationWidget(QString message){
    QMessageBox msgBox;
    msgBox.setWindowTitle("错误");
    msgBox.setText(message);
    msgBox.setIcon(QMessageBox::Information); // 设置对话框图标为错误
    msgBox.setStandardButtons(QMessageBox::Ok); // 只显示“确定”按钮
    msgBox.exec(); // 显示对话框
}

PlaneConstructor::PlaneConstructor() {}
CEntity* PlaneConstructor::create(QVector<CEntity*>& entitylist){
    //Constructor::create(entitylist);
    QVector<CPoint*>points;
    for(int i=0;i<entitylist.size();i++){
        CEntity* entity=entitylist[i];
        if(!entity->IsSelected())continue;
        if(entity->GetUniqueType()==enPoint){
            CPoint * point=(CPoint*)entity;
            points.push_back(point);
        }
    }
    //QVector<CPosition>&positions=Constructor::getPositions();//存储有效点
    if(points.size()==3){
        CPlane*plane=createPlane(*points[0],*points[1],*points[2]);
        plane->parent.push_back(points[0]);
        plane->parent.push_back(points[1]);
        plane->parent.push_back(points[2]);
        return plane;
    }
    if(points.size()<3){
        setWrongInformation(PointTooLess);
    }else if(points.size()>3){
        setWrongInformation(PointTooMuch);
    }
    return nullptr;
}
CPlane* PlaneConstructor::createPlane(CPosition p1,CPosition p2,CPosition p3){
    CPosition A=p1;
    CPosition B=p2;
    CPosition C=p3;

    QVector4D vecA(A.x, A.y, A.z, 1.0);
    QVector4D vecB(B.x, B.y, B.z, 1.0);
    QVector4D vecC(C.x, C.y, C.z, 1.0);

    QVector4D AB=vecA-vecB;
    QVector4D BC=vecB-vecC;
    QVector4D AC=vecA-vecC;

    CPosition center; // 平面的中心点
    QVector4D normal; // 平面的法向量

    QVector4D SquarePoint,AcutePoint1,AcutePoint2;
    QVector4D SquareEdge1,SquareEdge2,BevelEdge,long_edge_dir;
    double length,width;
    bool isSquare=true;
    int rightAngleVertex = findRightAngleVertex(vecA, vecB, vecC);
    if (rightAngleVertex == 1) {
        SquarePoint=vecA;
        AcutePoint1=vecB;
        AcutePoint2=vecC;
        //std::cout << "点A的角度近似为90度\n";
    } else if (rightAngleVertex == 2) {
        SquarePoint=vecB;
        AcutePoint1=vecA;
        AcutePoint2=vecC;
       // std::cout << "点B的角度近似为90度\n";
    } else if (rightAngleVertex == 3) {
        SquarePoint=vecC;
        AcutePoint1=vecB;
        AcutePoint2=vecA;
       // std::cout << "点C的角度近似为90度\n";
    } else {
        isSquare=false;
        SquarePoint=vecC;
        AcutePoint1=vecB;
        AcutePoint2=vecA;
        //InformationWidget("三点不构成直角");
        //std::cout << "没有点的角度近似为90度\n";
    }

        SquareEdge1=-SquarePoint+AcutePoint1;
        SquareEdge2=-SquarePoint+AcutePoint2;
        BevelEdge=AcutePoint1-AcutePoint2;
        //计算中心
        center=CPosition(SquarePoint.x()+SquareEdge1.x()/2+SquareEdge2.x()/2,
                           SquarePoint.y()+SquareEdge1.y()/2+SquareEdge2.y()/2,
                           SquarePoint.z()+SquareEdge1.z()/2+SquareEdge2.z()/2);
        normal=crossProduct(SquareEdge1,SquareEdge2);

        if(distance(SquareEdge1)>distance(SquareEdge2)){
            length=distance(SquareEdge1);
            width=distance(SquareEdge2);
            long_edge_dir=SquareEdge1;
        }else{
            length=distance(SquareEdge2);
            width=distance(SquareEdge1);
            long_edge_dir=SquareEdge2;
        }
        CPlane* newplane=new CPlane();
        newplane->setCenter(center);
        newplane->setNormal(normal);
        newplane->setDir_long_edge(long_edge_dir);
        newplane->setLength(length);
        newplane->setWidth(width);
        return newplane;
    // else{
    //     normal = crossProduct(AB,AC); // 计算法向量并归一化
    //     // 如果法向量的长度为零，说明三点共线
    //     if (normal.length() == 0) {
    //         setWrongInformation(PointTooClose);
    //         return  nullptr;
    //     }

    //     // 计算平面的中心
    //     center = CPosition((A.x + B.x + C.x) / 3, (A.y + B.y + C.y) / 3, (A.z + B.z + C.z) / 3);

    //     // 最长边向量
    //     long_edge_dir = QVector4D();// 方向向量可以根据具体应用调整

    //     double lenAB = distance(A, B);
    //     double lenAC = distance(A, C);
    //     double lenBC = distance(B, C);

    //     double width=qMin(lenAB,lenAC);
    //     width=qMin(width,lenBC);

    //     double length=qMax(lenAB,lenAC);
    //     length=qMax(length,lenBC);

    //     if(length==lenAB){
    //         long_edge_dir=AB;
    //     }else if(length==lenAC){
    //         long_edge_dir=AC;
    //     }else{
    //         long_edge_dir=BC;
    //     }

    //     CPlane* newplane=new CPlane();
    //     newplane->setCenter(center);
    //     newplane->setNormal(normal);
    //     newplane->setDir_long_edge(long_edge_dir);
    //     newplane->setLength(length);
    //     newplane->setWidth(width);
    //     return newplane;
    // }



}
CPlane* PlaneConstructor::createPlane(CPoint p1,CPoint p2,CPoint p3){
    CPosition pt1=p1.GetPt();
    CPosition pt2=p2.GetPt();
    CPosition pt3=p3.GetPt();
    return createPlane(pt1,pt2,pt3);
}
CPlane* PlaneConstructor::createPlane(CPosition posCenter, QVector4D normal, QVector4D direction, double length, double width){
    CPlane* newplane=new CPlane();
    newplane->setCenter(posCenter);
    newplane->setNormal(normal);
    newplane->setDir_long_edge(direction);
    newplane->setLength(length);
    newplane->setWidth(width);
    return newplane;
}
