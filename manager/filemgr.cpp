#include "filemgr.h"

FileMgr::FileMgr() {

}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr& FileMgr:: getCloudPtr(){
    return cloudptr;
}
QMap<QString, bool>& FileMgr::getModelFileMap(){
    return modelFileLMap;
}

QMap<QString, bool>& FileMgr::getMeasuredFileMap(){
    return measuredFileMap;
}

QMap<QString, bool>& FileMgr::getContentItemMap(){
    return contentItemMap;
}

QMap<QString, bool>& FileMgr::getIdentifyItemMap(){
    return identifyItemMap;
}

void FileMgr::removePointCloudKeys(QMap<QString, bool>& map)
{
    auto it = map.begin();
    while (it != map.end()) {
        if (it.key().startsWith("点云")) {
            it = map.erase(it);
        } else {
            ++it;
        }
    }
};
