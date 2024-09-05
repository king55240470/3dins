#ifndef COBJECT_H
#define COBJECT_H

#include "cshape.h"
#include <QString>

class CObject:public CShape
{
public:
    CObject();
    virtual ~CObject();//基类指针指向动态派生类的对象

    bool m_bMeasured;//是否被测量
    bool m_bChecked;//是否被检查
    bool m_bValid;//是否有效
    bool m_bBreak;//是否中断

    int m_nCsForm;//表示使用哪种坐标系

    QString m_strCName;// 存储名称
    QString m_strAutoName;// 存储自动生成的名称

    int m_nObjectID=0;//存储每个对象的编号
    static int m_nObjectCount;//表示一共创建了多少个对象

    bool IsMeasured();
    bool IsChecked();
    bool IsValid();
    bool IsBreak();

    void SetMeasured(bool);
    void SetChecked(bool);
    void SetValid(bool);
    void SetBreak(bool);

    virtual QString GetObjectCName();
    virtual QString GetObjectAutoName();

    void SetObjectCName(QString);
    void SetObjectAutoName(QString);

    int GetObjectID();

    bool IsFormAxis();
    bool IsFormOrigin();
    bool IsFormPlanar();

    virtual int GetState();
    virtual int IsNeedPause();
    virtual int InOOCRange();

    //序列化
    virtual QDataStream& serialize(QDataStream& out) const{
        out<<static_cast<const CShape&>(*this);//序列化基类部分
        out<<m_bMeasured
            <<m_bChecked
            <<m_bValid
            <<m_bBreak
            <<m_nCsForm
            <<m_strCName
            <<m_strAutoName
            <<m_nObjectID;
        return out;
    }
    //反序列化（反序列化通常不使用const成员函数，因为反序列化过程需要修改对象的状态以恢复其原始数据）
    virtual QDataStream& deserialize(QDataStream& in){
        in>>static_cast<CShape&>(*this);//反序列化基类部分
        in>>m_bMeasured
            >>m_bChecked
            >>m_bValid
            >>m_bBreak
            >>m_nCsForm
            >>m_strCName
            >>m_strAutoName
            >>m_nObjectID;
        return in;
    }
};

#endif // COBJECT_H
