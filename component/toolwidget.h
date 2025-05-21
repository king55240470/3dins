 #ifndef TOOLWIDGET_H
# define TOOLWIDGET_H
#include <QWidget>
#include <QToolBar>
#include <QAction>
#include <QVBoxLayout>
#include"toolaction.h"
#include <mainwindow.h>
#include <QStandardPaths>
#include"geometry/globes.h"
#include"geometry/centitytypes.h"
#include <QStack>
//槽函数
namespace tool_widget{
//Coord
void onCreateCoord();
void onSpinCoord();
void onSaveCoord();
//ViewAngle
void onFrontViewAngle();
void onIsometricViewAngle();
void onRightViewAngle();
void onUpViewAngle();
}

//用于提取监测点数据
struct MeasurementData {
    QString pointNumber;
    QString measuredValue;
    QString maxValue;
    QString minValue;
    QString angle;
    QString cameraCoordinates;
    QString isQualified;
    QString imagePath;
    QString type;
};
//用于提取点云尺寸测量数据
struct Size_MeasurementData{
    QString type;
    QString name;
    QString isVisible;//全局独有
    QString isColorful;
    QString isNormal;
    QString pointNumber;//全局独有
    QString max_error;
    QString min_error;
    QString average_error;
    QString num_triangularmesh;//局部独有
    QString three_dimensional;
    QString center_point;
    QString imagePath;//图片路径
};


class ToolWidget : public QWidget {
    Q_OBJECT
public:
    explicit ToolWidget(QWidget *parent = nullptr);

    MainWindow* m_pMainWin;

    void NotifySubscribe();

    static const int iconsize=30;
    static const int SingalToolBarActionNum=11;//每行工具栏的图标数目
    static const int ActionKindNum=5;//工具栏的种类数
    //获取图标名称
    QStringList* getSaveActionNames();
    QStringList* getConstructActionNames();
    QStringList* getFindActionNames();
    QStringList* getCoordActionNames();
    QStringList* getViewAngleActionNames();
    //获得图标的图片资源路径
    QStringList* getSaveIconPath();
    QStringList* getConstructIconPath();
    QStringList* getFindIconPath();
    QStringList* getCoordIconPath();
    QStringList* getViewAngleIconPath();

    int getToolbarNum();
    //获取图标数目
    int getSaveActionNum();
    int getConstructActionNum();
    int getCoordActionNum();
    int getFindActionNum();
    int getViewAngleActionNum();
    //清空工具栏
    void clearToolWidget();
    //初始化工具栏
    void createToolWidget();
    //添加图标
    int addSaveActions(QStringList& ,int,int=-1);
    int addConstructActions(QStringList& ,int,int=-1);
    int addFindActions(QStringList& ,int,int=-1);
    int addCoordActions(QStringList& ,int,int=-1);
    int addViewAngleActions(QStringList&,int,int=-1);

    ~ToolWidget();
    //将按钮与函数连接
    void connectActionWithF();
    void updateele();
    //添加入列表
    void addToList(CEntity*);
    void addToFindList(CEntity*);
    //得到构造实体列表
    QVector<CEntity*>& getConstructEntityList();
    //得到识别实体列表
    QVector<CEntity*>& getIdentifyEntityList();

    //serialize
    QDataStream& serializeEntityList(QDataStream& out, const QVector<CEntity*>& entityList);
    QDataStream& deserializeEntityList(QDataStream& in, QVector<CEntity*>& entityList);

    //返回UniqueToolBar的引用，以便contralwidget操作
    UniqueToolBar& getSave();
    UniqueToolBar& getFind();
    UniqueToolBar& getCoord();
    UniqueToolBar& getViewAngle();
    UniqueToolBar& getConstruct();
    ToolBarGathter& getToolBarGather();

    //获得保存图片路片路径
    QVector<QString>& getImagePaths();
    QVector<QString>& getImagePaths_part();
    //获得最近保存图片路径
    QString getlastCreatedImageFileFront();
    QString getlastCreatedImageFileTop();
    QString getlastCreatedImageFileRight();
    //获得当前时间编号
    QString getTimeString();
    //保存图片
    void SaveImage(QString path,std::string format="png");
    void SaveImage(CEntity* entity,Size_MeasurementData* PointCloudData=nullptr);
    QString getCompareImagePath();
    //初始化所有输出文件夹
    void InitOutputFolder();
    //一般工具
    QString getParentPath(int step=1);//获得上级路径
    void createFolder(QString path);//创建文件夹
    QString getOutputPath(QString kind);//获得输出路径

    //获得被选中的图标
    QAction * getAction_Checked(); //通过QAction->getObjectName得到名字
    //识别点是否选中
    bool IsFindPoint_Checked();


public slots:
              //构造
    void onConstructPoint();
    void onConstructLine();
    void onConstructCircle();
    void onConstructPlane();
    void onConstructRectangle();
    void onConstructCylinder();
    void onConstructCone();
    void onConstructSphere();
    void onConstructDistance();
    void onConstructPointCloud();
    void onConstructAngle();
    //识别
    void onFindPlane();
    void onFindPoint();
    void onFindLine();
    void onFindCircle();
    void onFindRectangle();
    void onFindCylinder();
    void onFindCone();
    void onFindSphere();
    //保存
    void onSavePdf();
    void onSaveExcel();
    void onSaveTxt();
    void onSaveWord();
    void onSaveImage();
    void onSavePointCloud();
    void createDistanceMeasurementReport();
    void createDistanceMeasurementReport_Pdf();

    //打开文件
    void onOpenExcel();
    void onOpenWord();
    void onOpenTxt();
    void onOpenPdf();
    void onOpenImage();

    //设置是否自动化
    void setauto(bool Auto);
    //用来保存最佳视角下拍摄图片的路径
    QMap<CEntity*,QString>& getCheckpoint_imagePath(){
        return m_checkpoint_imagePath;
    }

    void createFolderWithDialog();
    //用来提取点云，监测点数据
    void   ExtractData(QVector<CEntity *>& entitylist,QList<QList<QString>>& dataAll,QList<QString>& data_PointCloud);

    void   ExtractData(QVector<CEntity *>& entitylist,QList<QList<QString>>& dataAll,QList<QList<QString>>& data_accepted,QList<QList<QString>>& data_not_accepted);

    void   ExtractData(QVector<CEntity *>& entitylist,QList<QList<QString>>& dataAll);

    void   ExtractData(QVector<CEntity *>& entitylist,QList<Size_MeasurementData>&dataAll_size,Size_MeasurementData& data_pointCloud_size);
    void   saveAddress(const QString &address);
    void createFolder();
    QString loadAddress();
private:

    QString m_savePath="z:/results";//默认保存路径

    QMap<CEntity*,QString>m_checkpoint_imagePath;
    //对比点云保存路径
    QString CompareImagePath;
    //输出保存路径
    //保存打开的开关状态
    ToolAction* Action_Checked;
    bool Is_FindPoint_Cheked;


    void clearToolBar(QToolBar *toolbar);
    //新的工具栏类
    UniqueToolBar m_save;
    UniqueToolBar m_find;
    UniqueToolBar m_coord;
    UniqueToolBar m_viewAngle;
    UniqueToolBar m_construct;

    ToolBarGathter m_toolBarGather;

    //存储QAction
    ToolAction **  save_actions_;
    ToolAction **  construct_actions_;
    ToolAction **  find_actions_;
    ToolAction **  coord_actions_;
    ToolAction **  view_angle_actions_;
    QToolBar   **  toolBars;
    //路径
    QStringList save_action_iconpath_list_;
    QStringList construct_action_iconpath_list_;
    QStringList find_action_iconpath_list_;
    QStringList coord_action_iconpath_list_;
    QStringList view_angle_action_iconpath_list_;

    //名称
    QStringList save_action_name_list_;
    QStringList construct_action_name_list_;
    QStringList find_action_name_list_;
    QStringList coord_action_name_list_;
    QStringList view_angle_action_name_list_;
    //图标数目
    int m_nToolbarNum;
    int m_nSaveActionNum;
    int m_nConstructActionNum;
    int m_nFindActionNum;
    int m_nCoordActionNum;
    int m_nViewAngleActionNum;
    //存储被选择的内容索引
    QVector<int> m_point_index;
    QVector<CPoint*>m_selected_points;
    QVector<int> m_plane_index;
    QVector<CPlane*>m_selected_plane;

    //存储构建的元素
    QVector<CEntity*> constructEntityList;
    QVector<CEntity*> identifyEntityList;

    //存储全局对比过程中产生图片路径
    QVector<QString> imagePaths;
    //存储局部对比过程中产生的图片路径
    QVector<QString> imagePaths_part;

    //最新文件路径
    QString lastCreatedExcelFile;
    QString lastCreatedWordFile;
    QString lastCreatedTxtFile;
    QString lastCreatedPdfFile;
    QString lastCreatedImageFileFront;
    QString lastCreatedImageFileTop;
    QString lastCreatedImageFileRight;

    //是否自动化保存内容
    bool IsAuto=false;
    //是否已保存
    bool m_savePdf=false;
    bool m_saveWord=false;
    bool m_saveTxt=false;
    bool m_saveExcel=false;



};
#endif // TOOLWIDGET_H
