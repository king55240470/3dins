#ifndef CENTITY_H
#define CENTITY_H

#include "cobject.h"
#include "globes.h"
#include "CPcs.h"
#include <QVector>
#include <vtkSmartPointer.h>
#include <vtkActor.h>
#include <pcl/io/auto_io.h>

class CEntity:public CObject
{
public:
    CPcs* m_pRefCoord;//参考坐标系
    CPcs* m_pCurCoord;//当前坐标系
    CPcs* m_pExtCoord;

    QVector<CEntity*> m_ConstructList; // 元素的构建列表,该元素由其他元素构建而成
    //QVector<CImageTool *>  m_ToolArray;//存储该实体使用的工具

    bool m_bShowCNameLabel;//是否显示该实体的名称

    int  m_nRef;//表示参考哪个坐标系，m_nRef表示坐标系的ID

    bool m_bRemeasure;//是否需要重新测量

    enum CREATE_FORM m_CreateForm;
    enum ENTITY_TYPE m_EntityType;
    enum COMPENSATE_TYPE m_CompenType;
public:
    CEntity();
    virtual ~CEntity(){}
    CPcs* GetRefCoord() override;//使用override关键字时，如果派生类的函数签名与基类的虚函数不匹配，编译器会报错,这有助于确保正确地重写基类函数
    CPcs* GetCurCoord() override;
    CPcs* GetExtCoord() override;

    void SetRefCoord(CPcs *)override;
    void SetCurCoord(CPcs *)override;
    void SetExtCoord(CPcs *)override;

    virtual QString getCEntityInfo(){return nullptr;}; // 获取图形的信息，在浮动窗口显示

    QVector<CEntity*>GetConstructList();
    //void SetImageToolList(QVector<CImageTool *>);
    //QVector<CImageTool *> GetImageToolList();

    bool IsShowCNameLabel();

    int AddRef(CShape*);
    int DelAllRefShape();

    void SetRemeasure(bool);

    bool IsCanAutoMeasure();//是否自动测量
    bool IsMeasureByImage();//是否通过图像测量
    bool IsMeasureByLaser();//是否通过激光测量
    bool IsMeasureByProbe();//是否通过探针测量

    bool IsType(ENTITY_TYPE);
    // 当遍历entitylist进行绘制时，传入子类的指针，自动调用它们自己的draw
    virtual vtkSmartPointer<vtkActor> draw() {return nullptr;};

    void setEntityType(ENTITY_TYPE);
    ENTITY_TYPE getEntityType() const;

    void SetCompenType(COMPENSATE_TYPE);
    COMPENSATE_TYPE GetCompenType();

    void SetCreateForm(CREATE_FORM);
    CREATE_FORM GetCreateForm();


    //序列化
    QDataStream& serialize(QDataStream& out) const override;
    //反序列化
    QDataStream& deserialize(QDataStream& in) override;
};

#endif // CENTITY_H
