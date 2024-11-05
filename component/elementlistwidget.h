#ifndef ELEMENTLISTWIDGET_H
#define ELEMENTLISTWIDGET_H
#include "geometry/centity.h"
#include <QWidget>
#include <QLabel>
#include <QPushButton>
#include <QMenu>
#include <QContextMenuEvent>
#include <QVBoxLayout>
#include <QMessageBox>
#include <QLineEdit>
#include <QHBoxLayout>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QApplication>
#include <QDebug>
#include <QContextMenuEvent>
#include<QToolBar>
#include <QVector>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QAction>
#include "toolwidget.h"
#include "mainwindow.h"
#include "DataWidget.h"
#include "toolwidget.h"
class ElementListWidget : public QWidget
{
    Q_OBJECT
public:
    explicit ElementListWidget(QWidget *parent = nullptr);
    void CreateEllipse(CObject*);
    void onDeleteEllipse();
    int getNextId();
    void onCustomContextMenuRequested(const QPoint &pos);
    void deal_actionNew_triggered();
    void judgetype(CEntity*);
    void removeall();
    void updateInsertIndicatorPosition();
    void upadteelementlist();
    QList<QTreeWidgetItem*> getSelectedItems();
    QVector<CObject*> getEleobjlist();
    void selectall();
    void showDialog();
    void selectcommonitem();
    void onItemClicked();
    void setTolerance();
    void BtnClicked();
    void ShowParent(CObject*obj);
protected:
    void keyPressEvent(QKeyEvent *event) override;
    void keyReleaseEvent(QKeyEvent *event) override;
private:
    QTreeWidget *treeWidgetNames;
    QTreeWidget *treeWidgetInfo;
    QLineEdit *xLineEdit;
    QLineEdit *yLineEdit;
    QLineEdit *sizeLineEdit;
    int nextEllipseIndex=0;
    QToolBar * toolBar;
    CEntity * centity;
    MainWindow *m_pMainWin=nullptr;
    static int pcscount;
    QVector<CObject*>eleobjlist;
    //std::unordered_map<QTreeWidgetItem*, size_t> itemToIndexMap;
    bool ctrlPressed = false;
    QLineEdit* up;
    QLineEdit* down;
    QPushButton *updownBtn;
    QDialog *dialog;
signals:
    void itemSelected(int index);
};

#endif // ELEMENTLISTWIDGET_H
