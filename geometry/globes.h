#ifndef GLOBES_H
#define GLOBES_H

#include <QDataStream>


enum CREATE_FORM : int
{
    eMeasure = 0x0,
    eConstruct = 0x1,
    ePreset = 0x2,
    eCopy = 0x3
};

enum ENTITY_TYPE : int
{
    enNone          = -1,
    enPoint         = 0x0,
    enLine          = 0x1,
    enArc           = 0x2,
    enCircle        = 0x3,
    enEllipse       = 0x4,
    enSlot          = 0x5,
    enRing          = 0x6,
    enRectangle     = 0x7,
    enAngle         = 0x8,
    enDistance      = 0x9,
    enSpline        = 0xA,
    enFocusPlane    = 0xB,
    enHeight        = 0xC,
    enInflexion     = 0xD,
    enPlane         = 0xE,
    enSphere        = 0xF,
    enCylinder      = 0x10,
    enCone          = 0x11,
    enOpenCurve     = 0x12,
    enCloseCurve    = 0x13,
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
};

enum COMPENSATE_TYPE : int
{
    eCompensateHit = 0x0,
    eCompensateAuto = 0x1,
    eCompensateX = 0x2,
    eCompensateY = 0x3,
    eCompensateZ = 0x4,
    eCompensateNone = 0x5
};

/*class CPosition{
private:
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
    friend QDataStream& operator<<(QDataStream& out,const CPosition& pos){
        out<<pos.x<<pos.y<<pos.z;
        return out;
    }
    friend QDataStream& operator>>(QDataStream& in,CPosition& pos){
        in>>pos.x>>pos.y>>pos.z;
        return in;
    }
};*/

#endif // GLOBES_H
