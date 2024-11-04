#ifndef CHOSENCENTITYMGR_H
#define CHOSENCENTITYMGR_H

#include "geometry/centitytypes.h"

class ChosenCEntityMgr
{
public:
    ChosenCEntityMgr();

    // 获取ChosenCEntityList
    QVector<CPosition>& getChosenCEntityList(){
        return ChosenCEntityList;
    }

    // 根据选中的坐标创建一个CPosition，并存入ChosenCEntityList
    void CreatPosition(double pos[3]);

    // 遍历ChosenCEntityList，由取消选中的坐标删除对应的Position
    void DeletePosition(double pos[3]);

private:
    QVector<CPosition> ChosenCEntityList; // 存储从窗口中选中的图形，目前只有点
};

#endif // CHOSENCENTITYMGR_H
