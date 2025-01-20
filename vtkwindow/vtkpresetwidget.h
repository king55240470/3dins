#ifndef VTKPRESETWIDGET_H
#define VTKPRESETWIDGET_H

#include <QTreeWidget>
#include <QVBoxLayout>

class MainWindow;
class VtkPresetWidget : public QWidget
{
    Q_OBJECT
public:
    explicit VtkPresetWidget(QWidget *parent = nullptr);
    void setWidget(QString);
    void showContextMenu(const QPoint &);
    void deleteWidget();

private:
    MainWindow *m_pMainWin;
    QVBoxLayout *layout;
    QTreeWidget *treeWidget;
    QMenu *contextMenu;//右键菜单
    QAction *deleteAction;
signals:
};

#endif // VTKPRESETWIDGET_H
