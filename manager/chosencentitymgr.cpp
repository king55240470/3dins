#include "chosencentitymgr.h"

ChosenCEntityMgr::ChosenCEntityMgr() {}

// 根据选中的坐标创建点，并存入ChosenCEntityList
void ChosenCEntityMgr::CreatPosition(double pos[3])
{
    std::cout << "Picked point: " << pos[0] << ", " << pos[1] << ", " << pos[2] << std::endl;

    auto newpoint = CPosition(pos[0], pos[1], pos[2]);
    // 将新创建的点存入ChosenCEntityList
    getChosenCEntityList().append(newpoint);
}

// 遍历ChosenCEntityList，由取消选中的坐标删除对应的CPoint
void ChosenCEntityMgr::DeletePosition(double pos[3])
{
    std::cout << "Picked point: " << pos[0] << ", " << pos[1] << ", " << pos[2] << std::endl;

    CPosition targetpoint = CPosition(pos[0], pos[1], pos[2]);
    for(auto i = 0;i < ChosenCEntityList.size();i++){
        if(ChosenCEntityList[i].x == targetpoint.x
            && ChosenCEntityList[i].y == targetpoint.y
            && ChosenCEntityList[i].z == targetpoint.z)
            ChosenCEntityList.remove(i);
    }
}
