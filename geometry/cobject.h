#ifndef COBJECT_H
#define COBJECT_H
#include <QString>
#include "cshape.h"

class CObject : public CShape
{

private:
    QString m_strCName;// 存储名称
    QString m_strAutoName;// 存储自动生成的名称

    int m_nObjectID=0;//存储每个对象的编号
    //static int m_nObjectCount;//表示一共创建了多少个对象

public:
    CObject(QString name,QString Autoname,int id=0):m_strCName(name),m_strAutoName(Autoname),m_nObjectID(id)
    {
    };
    QString getname(){
        return this->m_strCName;
    };
};

#endif // COBJECT_H
