#ifndef GLOBES_H
#define GLOBES_H

#include <QDataStream>

//用什么方式创建
enum CREATE_FORM : int
{
    eMeasure = 0x0,//测量
    eConstruct = 0x1,//
    ePreset = 0x2,//预设
    eCopy = 0x3,//复制
    eReconstruct = 0x4//重建
};

enum ENTITY_TYPE : int
{
    enNone          = -1,
    enPoint         = 0x0,//点
    enLine          = 0x1,//线
    enArc           = 0x2,//弧
    enCircle        = 0x3,//圆
    enEllipse       = 0x4,//椭圆
    enSlot          = 0x5,//槽
    enRing          = 0x6,//环
    enRectangle     = 0x7,//矩形
    enAngle         = 0x8,//夹角
    enDistance      = 0x9,//距离
    enSpline        = 0xA,//曲线
    enFocusPlane    = 0xB,//焦点
    enHeight        = 0xC,//高度
    enInflexion     = 0xD,//拐点
    enPlane         = 0xE,//面
    enSphere        = 0xF,//球
    enCylinder      = 0x10,//圆柱
    enCone          = 0x11,//圆锥
    enOpenCurve     = 0x12,//开放曲线
    enCloseCurve    = 0x13,//封闭曲线
    enCalculation   = 0x14,
    en3Pdistance    = 0x15,
    en4Pangle       = 0x16,
    enPattern       = 0x17,
    enGoldfinger    = 0x18,
    enFlaw          = 0x19,
    enImageContrast = 0x1A,
    enPointCloud    = 0x1B,
    enCable         = 0x1C,
    enShaft         = 0x1D,
    enInputOutput   = 0x1E,
    enOutputIO      = 0x1F,
    enRegion        = 0x20,
    enScanCode      = 0x21,
    enTotalCount    = 0x22,
    enCuboid        = 0x23,
    enSurfaces      = 0x24,
};

//校正偏差
enum COMPENSATE_TYPE : int
{
    eCompensateHit = 0x0,
    eCompensateAuto = 0x1,
    eCompensateX = 0x2,
    eCompensateY = 0x3,
    eCompensateZ = 0x4,
    eCompensateNone = 0x5
};

enum RELY_ON_CS_TYPE : int
{
    csCur=0,
    csRef=1
};

class CPosition{
public:
    double x;
    double y;
    double z;
public:
    CPosition(){
        x=y=z=0;
    }
    CPosition(double x1,double y1,double z1){
        x=x1;
        y=y1;
        z=z1;
    }
    CPosition& operator=(const CPosition& obj){
        if(this==&obj){
            return *this;
        }
        x=obj.x;
        y=obj.y;
        z=obj.z;
        return *this;
    }
    friend QDataStream& operator<<(QDataStream& out, const CPosition& pos){
        out<<pos.x<<pos.y<<pos.z;
        return out;
    }
    friend QDataStream& operator>>(QDataStream& in,CPosition& pos){
        in>>pos.x>>pos.y>>pos.z;
        return in;
    }
};

#endif // GLOBES_H
