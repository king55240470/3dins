#ifndef FILEMGR_H
#define FILEMGR_H

#include <qmap>
#include <qvector>

class FileMgr
{
public:
    FileMgr();
    QMap<QString, bool>& getModelFileMap();
    QMap<QString, bool>& getMeasuredFileMap();
    QMap<QString, bool>& getContentItemMap();
    QMap<QString, bool>& getIdentifyItemMap();
private:
    QMap<QString, bool> modelFileLMap;
    QMap<QString, bool> measuredFileMap;
    QMap<QString, bool> contentItemMap;
    QMap<QString, bool> identifyItemMap;
};

#endif // FILEMGR_H
