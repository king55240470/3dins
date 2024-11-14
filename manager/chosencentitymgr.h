#ifndef CHOSENCENTITYMGR_H
#define CHOSENCENTITYMGR_H

#include "geometry/centitytypes.h"

class ChosenCEntityMgr
{
public:
    ChosenCEntityMgr();

    // 获取ChosenCEntityList
    QVector<CPosition>& getChosenActorAxes(){
        return chosenActorAxes;
    }
    QVector<CEntity*>& getChosenCEntityList(){
        return chosenCEntityList;
    }

    // 根据选中的坐标创建一个CPosition，并存入ChosenCEntityList
    void CreatPosition(double pos[3]);

    // 遍历ChosenCEntityList，由取消选中的坐标删除对应的Position
    void DeletePosition(double pos[3]);

private:
    QVector<CPosition> chosenActorAxes; // 存储从窗口中选中actor时的点击坐标
    QVector<CEntity*> chosenCEntityList; // 存储从窗口中选中的图形
};

#endif // CHOSENCENTITYMGR_H
