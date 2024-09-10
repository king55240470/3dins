#ifndef CENTITYTYPES_H
#define CENTITYTYPES_H

#include "centity.h"
#include <QSettings>
#include "cposition.h"
class CLine  : public CEntity
{
public:
    double m_k;
    double m_b;

    static int lineCount;
    static int currentLineId;

    double k;
    double b;


    CPosition begin;
    CPosition end;

public:
    CLine()
    {
        k=0;
        b=0;
        begin.x=0;
        begin.y=0;
        begin.z=0;
        end.x=0;
        end.y=0;
        end.z=0;
        currentLineId = ++lineCount;
        m_strAutoName = QString("线%1").arg(currentLineId);
        m_strCName = QString("线%1").arg(currentLineId);
    }
    int GetUniqueType() override {

        return enLine;
    }
    void SetPosition(CPosition pt1,CPosition pt2)
    {
        begin = pt1;
        end = pt2;
    }
    CPosition getPosition1()
    {
        return begin;
    }
    CPosition getPosition2()
    {
        return end;
    }
    int getId()  {
        return currentLineId; // 返回该点对象的唯一标识符
    }
    CPosition GetObjectCenterLocalPoint()
    {
        CPosition pos;
        pos.x = (begin.x+end.x)/2;
        pos.y = (begin.y+end.y)/2;
        pos.z = (begin.z+end.z)/2;
        return pos;
    }
    CPosition GetObjectCenterGlobalPoint()
    {
        CPosition globalBeginPos, globalEndPos, globalCenterPos;
        //globalBeginPos =  GetWorldPcsPos(begin);
        //globalEndPos =  GetWorldPcsPos(end);
        globalCenterPos.x = (globalBeginPos.x+globalEndPos.x)/2;
        globalCenterPos.y = (globalBeginPos.y+globalEndPos.y)/2;
        globalCenterPos.z = (globalBeginPos.z+globalEndPos.z)/2;
        return globalCenterPos;
    }
    CPosition getBegin() const;
    void setBegin(const CPosition &newBegin);
    CPosition getEnd() const;
    void setEnd(const CPosition &newEnd);
};

class CPoint : public CEntity
{
public:
    CPosition m_pt;
    static int pointCount;
    int currentPointId;
public:
    CPoint()
    {
        m_pt.x=0;
        m_pt.y=0;
        m_pt.z=0;
        currentPointId = ++pointCount;
        m_strAutoName = QString("点%1").arg(currentPointId);
        m_strCName = QString("点%1").arg(currentPointId);
    }
    int GetUniqueType() override{
        return enPoint;
    }

    void SetPosition(CPosition pt)
    {
        m_pt = pt;
    }
    int getId()  {
        return currentPointId; // 返回该点对象的唯一标识符
    }


    CPosition GetPt()
    {
        return m_pt;
    }

    CPosition GetObjectCenterLocalPoint()
    {
        return m_pt;
    }
    CPosition GetObjectCenterGlobalPoint()
    {
        return GetWorldPcsPos(m_pt);
    }

};


class CCircle  : public CEntity
{
    CPosition m_pt;

    double m_d;
    static int circleCount;
    int currentCircleId;
    QDataStream& serialize(QDataStream& out) const override {
        CEntity::serialize(out);  // 先序列化基类部分
        out << m_pt << m_d << circleCount << currentCircleId;  // 序列化CCircle特有部分
        out <<constructPt1<<constructPt2<<constructPt3;
        return out;
    }

    QDataStream& deserialize(QDataStream& in) override {
        CEntity::deserialize(in);  // 先反序列化基类部分
        in >> m_pt >> m_d >> circleCount >> currentCircleId;  // 反序列化CCircle特有部分
        in>>constructPt1>>constructPt2>>constructPt3;
        return in;
    }
public:
    CPosition constructPt1,constructPt2,constructPt3;
    CCircle()
    {
        m_pt.x=0;
        m_pt.y=0;
        m_pt.z=0;
        m_d =0 ;
        currentCircleId = ++circleCount;
        m_strAutoName = QString("圆%1").arg(currentCircleId);
        m_strCName = QString("圆%1").arg(currentCircleId);
    }
    void SetDiameter(double d);
    void SetCenter(CPosition pt);
    CPosition getCenter();
    double getDiameter();
    int GetUniqueType() override{
        return enCircle;
    }
    static int getCircleCount() {
        return circleCount;
    }
    int getId()  ;

    // void ChangePosBeforChangeRefCoord(CPcs *pNewRefCoord)override;
    CPosition GetObjectCenterPoint() ;

    void SetNominal() ;
    void ChangePosBeforChangeRefCoord(CPcs *pNewRefCoord){
        bool bInv=false;
        QVector4D posVec =pNewRefCoord->m_mat.inverted(&bInv) * m_pRefCoord->m_mat * QVector4D(m_pt.x, m_pt.y, m_pt.z, 1);
        m_pt.x = posVec.x();
        m_pt.y = posVec.y();
        m_pt.z = posVec.z();

    }
    CPosition GetObjectCenterLocalPoint()
    {
        return m_pt;
    }
    CPosition GetObjectCenterGlobalPoint()
    {
        return GetWorldPcsPos(m_pt);
    }
};

class CPlane : public CEntity
{
private:
    CPosition center;
    QVector4D normal;
    QVector4D dir_long_edge;    // dir 表示方向direction
    double length;
    double width;

    static int plainCount;
    int currentPlainId;

public:
    CPosition getCenter() const;
    void setCenter(const CPosition &newCenter);
    QVector4D getNormal() const;
    void setNormal(const QVector4D &newNormal);
    QVector4D getDir_long_edge() const;
    void setDir_long_edge(const QVector4D &newDir_long_edge);
    double getLength() const;
    void setLength(double newLength);
    double getWidth() const;
    void setWidth(double newWidth);

    CPlane(){
        center.x=0;
        center.y=0;
        center.z=0;
        normal=QVector4D(0,0,0,1);
        currentPlainId = ++plainCount;
        m_strAutoName = QString("平面%1").arg(currentPlainId);
        m_strCName = QString("平面%1").arg(currentPlainId);
    }

    int GetUniqueType() override {

        return enPlane;
    }
    CPosition GetObjectCenterLocalPoint()
    {
        return center;
    }
    CPosition GetObjectCenterGlobalPoint()
    {
        return GetWorldPcsPos(center);
    }
};

class CSphere : public CEntity{

    CPosition center;
    double diameter;

    static int sphereCount;
    int currentSphereId;

public:
    CPosition getCenter() const;
    void setCenter(const CPosition &newCenter);
    double getDiameter() const;
    void setDiameter(double newDiameter);

    CSphere(){
        center.x=0;
        center.y=0;
        center.z=0;
        currentSphereId = ++sphereCount;
        m_strAutoName = QString("球%1").arg(currentSphereId);
        m_strCName = QString("球%1").arg(currentSphereId);
    }
    int GetUniqueType() override{

        return enSphere;
    }
    CPosition GetObjectCenterLocalPoint()
    {
        return center;
    }
    CPosition GetObjectCenterGlobalPoint()
    {
        return GetWorldPcsPos(center);
    }
};


class CCylinder : public CEntity{
    QVector4D axis;
    double diameter;
    double height;
    CPosition btm_center;

    static int cylinderCount;
    int currentCylinderId;


public:
    CCylinder(){
        btm_center.x=0;
        btm_center.y=0;
        btm_center.z=0;
        currentCylinderId = ++cylinderCount;
        m_strAutoName = QString("圆柱%1").arg(currentCylinderId);
        m_strCName = QString("圆柱%1").arg(currentCylinderId);
    }

    QVector4D getAxis() const;
    void setAxis(const QVector4D &newAxis);
    double getDiameter() const;
    void setDiameter(double newDiameter);
    double getHeight() const;
    void setHeight(double newHeight);
    CPosition getBtm_center() const;
    void setBtm_center(const CPosition &newBtm_center);

    int GetUniqueType() override {

        return enCylinder;
    }
    CPosition GetObjectCenterLocalPoint()
    {
        return btm_center;
    }
    CPosition GetObjectCenterGlobalPoint()
    {
        return GetWorldPcsPos(btm_center);
    }
};

class CCone : public CEntity{

    QVector4D axis;
    double radian;
    double height;
    double cone_height;
    CPosition vertex;

    static int coneCount;
    int currentConeId;

public:
    CCone(){
        vertex.x=0;
        vertex.y=0;
        vertex.z=0;
        currentConeId = ++coneCount;
        m_strAutoName = QString("圆柱%1").arg(currentConeId);
        m_strCName = QString("圆柱%1").arg(currentConeId);
    }
    int GetUniqueType() override {
        return enCone;
    }
    CPosition GetObjectCenterLocalPoint()
    {
        return vertex;
    }
    CPosition GetObjectCenterGlobalPoint()
    {
        return GetWorldPcsPos(vertex);
    }

public:
    QVector4D getAxis() const;
    void setAxis(const QVector4D &newAxis);
    double getRadian() const;
    void setRadian(double newRadian);
    double getHeight() const;
    void setHeight(double newHeight);
    double getCone_height() const;
    void setCone_height(double newCone_height);
    CPosition getVertex() const;
    void setVertex(const CPosition &newVertex);
};

#endif // CENTITYTYPES_H
