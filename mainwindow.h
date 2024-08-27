#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QSplitter>
#include <QMenuBar>
#include <QStatusbar>

class DataWidget;
class ElementListWidget;
class FileManagerWidget;
class ToolWidget;
class UnknownWidget;
class VtkWidget;
class ReportWidget;
class LogWidget;

class MainWindow : public QMainWindow
{
    Q_OBJECT
private:
    QSplitter *spMainWindow;
    QSplitter *spLeft;
    QSplitter *spMiddleup;
    QSplitter *spMiddledown;
    QSplitter *spRightup;
    QSplitter *spRightdown;

    DataWidget *pWinDataWidget;
    ElementListWidget *pWinElementListWidget;
    FileManagerWidget *pWinFileManagerWidget;
    ToolWidget *pWinToolWidget;
    UnknownWidget *pWinUnknownWidget;
    VtkWidget *pWinVtkWidget;
    ReportWidget *pWinReportWidget;
    LogWidget *pWinLogWidget;

    QMenuBar * bar;
    QStatusBar *stbar;


public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void LoadWidgets();
    void RestoreWidgets();
    void setupUi();
};
#endif // MAINWINDOW_H
