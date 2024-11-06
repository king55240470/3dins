#include "filemgr.h"

FileMgr::FileMgr() {

}
pcl::PointCloud<pcl::PointXYZ>::Ptr& FileMgr:: getCloudPtr(){
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
