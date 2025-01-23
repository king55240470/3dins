#ifndef COBJECT_H
#define COBJECT_H
#include <QString>
#include "cshape.h"
#include <QString>
#include <vtkSmartPointer.h>
#include <vtkAxesActor.h>

class CObject:public CShape
{

public:
    QString m_strCName;// 存储名称
    QString m_strAutoName;// 存储自动生成的名称

    int m_nObjectID;//存储每个对象的编号
    int m_nObjectCount;//表示一共创建了多少个对象
    bool m_bMeasured;//是否被测量
    bool m_bChecked;//是否被检查
    bool m_bValid;//是否有效
    bool m_bBreak;//是否中断

    int m_nCsForm;//表示使用哪种坐标系
    QVector<CObject*>parent;
    QString Form;
public:
    CObject();
    virtual ~CObject() {}
    //基类指针指向动态派生类的对象

    void AddObjectCount();
    void SetID(int id);

    bool IsMeasured();
    bool IsChecked();
    bool IsValid();
    bool IsBreak();

    void SetMeasured(bool);
    void SetChecked(bool);
    void SetValid(bool);
    void SetBreak(bool);

    QString GetObjectCName();
    QString GetObjectAutoName();
    CPosition GetObjectCenterGlobalPoint();
    CPosition GetObjectCenterLocalPoint();
    CPosition GetWorldPcsPos(CPosition);

    void SetObjectCName(QString);
    void SetObjectAutoName(QString);

    int GetObjectID();

    bool IsFormAxis();
    bool IsFormOrigin();
    bool IsFormPlanar();

    int GetState();
    int IsNeedPause();
    int InOOCRange();

    //序列化
    virtual QDataStream& serialize(QDataStream& out) const;
    //反序列化（反序列化通常不使用const成员函数，因为反序列化过程需要修改对象的状态以恢复其原始数据）
    virtual QDataStream& deserialize(QDataStream& in);

    // 用于绘制坐标系的draw，在cpcsnode中实现，默认参数是为了让entity重载
    virtual vtkSmartPointer<vtkAxesActor> draw(int x = 0) {return nullptr;};
};

#endif // COBJECT_H
