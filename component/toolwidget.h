 #ifndef TOOLWIDGET_H
# define TOOLWIDGET_H
#include <QWidget>
#include <QToolBar>
#include <QAction>
#include <QVBoxLayout>
#include"toolaction.h"
#include <mainwindow.h>
#include"geometry/globes.h"
#include"geometry/centitytypes.h"
//槽函数
namespace tool_widget{
//Find
void onFindPoint();
void onFindLine();
void onFindCircle();
void onFindPlan();
void onFindRectangle();
void onFindCylinder();
void onFindCone();
void onFindSphere();
//Construct
void onConstructPoint();
void onConstructLine();
void onConstructCircle();
void onConstructPlane();
void onConstructRectangle();
void onConstructCylinder();
void onConstructCone();
void onConstructSphere();
//Coord
void onCreateCoord();
void onSpinCoord();
void onSaveCoord();
//Save
void onSavePdf();
void onSaveExcel();
void onSaveTxt();
void onSaveWord();
void onSaveImage();
//ViewAngle
void onFrontViewAngle();
void onIsometricViewAngle();
void onRightViewAngle();
void onUpViewAngle();
}

class ToolWidget : public QWidget {
    Q_OBJECT
public:
    explicit ToolWidget(QWidget *parent = nullptr);

    MainWindow* m_pMainWin;

    void NotifySubscribe();

    static const int SingalToolBarActionNum=9;//每行工具栏的图标数目
    static const int ActionKindNum=5;//工具栏的种类数
    //获取图标名称
    QStringList* getSaveActionNames();
    QStringList* getConstructActionNames();
    QStringList* getFindActionNames();
    QStringList* getCoordActionNames();
    QStringList* getViewAngleActionNames();

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
    void connectActionWithF();
    void updateele();

    void addToList(CEntity*);

    QVector<CEntity*>& getConstructEntityList();

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
    void onFindPlane();


private:
    void clearToolBar(QToolBar *toolbar);
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
};
#endif // TOOLWIDGET_H
