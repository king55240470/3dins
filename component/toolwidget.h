 #ifndef TOOLWIDGET_H
# define TOOLWIDGET_H
#include <QWidget>
#include <QToolBar>
#include <QAction>
#include <QVBoxLayout>
#include"toolaction.h"
#include <mainwindow.h>

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
void onConstructPlan();
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
}

class ToolWidget : public QWidget {
    Q_OBJECT
public:
    explicit ToolWidget(QWidget *parent = nullptr);

    MainWindow* m_pMainWin;

    void NotifySubscribe();

    static const int SingalToolBarActionNum=9;
    static const int ActionKindNum=4;

    QStringList* getSaveActionNames();
    QStringList* getConstructActionNames();
    QStringList* getFindActionNames();
    QStringList* getCoordActionNames();

    int getToolbarNum();

    int getSaveActionNum();
    int getConstructActionNum();
    int getCoordActionNum();
    int getFindActionNum();

    void clearToolWidget();

    void createToolWidget();

    int addSaveActions(QStringList& ,int,int=-1);
    int addConstructActions(QStringList& ,int,int=-1);
    int addFindActions(QStringList& ,int,int=-1);
    int addCoordActions(QStringList& ,int,int=-1);

     ~ToolWidget();
    void connectActionWithF();


private:
    void clearToolBar(QToolBar *toolbar);
    ToolAction **  save_actions_;
    ToolAction **  construct_actions_;
    ToolAction **  find_actions_;
    ToolAction **  coord_actions_;
    QToolBar   **  toolBars;

    QStringList save_action_iconpath_list_;
    QStringList construct_action_iconpath_list_;
    QStringList find_action_iconpath_list_;
    QStringList coord_action_iconpath_list_;


    QStringList save_action_name_list_;
    QStringList construct_action_name_list_;
    QStringList find_action_name_list_;
    QStringList coord_action_name_list_;

    int m_nToolbarNum;
    int m_nSaveActionNum;
    int m_nConstructActionNum;
    int m_nFindActionNum;
    int m_nCoordActionNum;

};
#endif // TOOLWIDGET_H
