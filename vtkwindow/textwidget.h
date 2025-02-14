// #ifndef TEXTDISPLAYMANAGER_H
// #define TEXTDISPLAYMANAGER_H
// #include "vtkwindow/vtkwidget.h"

// #include <QWidget>
// #include <QGraphicsScene>
// #include <QGraphicsTextItem>
// #include <QGraphicsRectItem>
// #include <QGraphicsLineItem>
// #include <QMap>
// #include <QPointer>

// class CEntity;

// class TextDisplayManager : public QWidget {
//     Q_OBJECT

// public:
//     explicit TextDisplayManager(QWidget *parent = nullptr);
//     ~TextDisplayManager();

//     void createText(CEntity *entity);
//     void closeTextActor(CEntity *entity);
//     void updateTextPosition(CEntity *entity, const QPointF &position);
//     void closeAllText();

// private:
//     QGraphicsScene *scene;
//     QMap<CEntity *, QGraphicsTextItem *> entityToTextItems;
//     QMap<CEntity *, QGraphicsRectItem *> entityToTextBoxs;
//     QMap<CEntity *, QGraphicsLineItem *> entityToLines;
//     QMap<CEntity *, QGraphicsTextItem *> entityToTitleTextItems;

//     QGraphicsTextItem *createTitleTextItem(const QString &text, const QPointF &position);
//     QGraphicsRectItem *createTextBox(QGraphicsTextItem *textItem, const QPointF &position);
//     QGraphicsLineItem *createLine(CEntity *entity, QGraphicsTextItem *textItem);
//     QGraphicsTextItem *createCloseIcon(const QPointF &position);
//     void updateLine(CEntity *entity, QGraphicsTextItem *textItem);
// };

// #endif // TEXTDISPLAYMANAGER_H
