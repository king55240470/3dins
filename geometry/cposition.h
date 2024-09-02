#ifndef CPOSITION_H
#define CPOSITION_H

#include<QDataStream>

class CPosition
{
public:
    double x;
    double y;
    double z;
public:

    CPosition(){x=y=z=0;};
    CPosition(double x1, double y1, double z1){
        x=x1;
        y=y1;
        z=z1;
    }

    CPosition& operator=(const CPosition& other) {
        if (this == &other) {
            return *this;
        }

        this->x = other.x;
        this->y = other.y;
        this->z = other.z;

        return *this;
    }

    //序列化/反序列化CPosition对象
    friend QDataStream& operator<<(QDataStream& out, const CPosition& pos) {
        out << pos.x << pos.y << pos.z;
        return out;
    }

    friend QDataStream& operator>>(QDataStream& in, CPosition& pos) {
        in >> pos.x >> pos.y >> pos.z;
        return in;
    }


};

#endif // CPOSITION_H
