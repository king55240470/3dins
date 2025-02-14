// #include "vtkwindow/textwidget.h"
// #include <QGraphicsEllipseItem>
// #include <QGraphicsView>
// #include <QPainter>

// TextDisplayManager::TextDisplayManager(QWidget *parent) : QWidget(parent) {
//     scene = new QGraphicsScene(this);
//     QGraphicsView *view = new QGraphicsView(scene, this);
//     view->setGeometry(0, 0, this->width(), this->height());
//     view->setAlignment(Qt::AlignLeft | Qt::AlignTop);
// }

// // 析构函数
// TextDisplayManager::~TextDisplayManager() {
//     closeAllText();
// }

// // 创建文本标注
// void TextDisplayManager::createText(CEntity *entity) {
//     if (!entity) return;

//     if (entityToTextItems.contains(entity)) return;

//     QString qstr = entity->getCEntityInfo();
//     QString firstLine = qstr.section('\n', 0, 0);
//     QString remainingText = qstr.section('\n', 1);

//     QGraphicsTextItem *textItem = scene->addText(remainingText);
//     textItem->setFont(QFont("Times", 16));
//     textItem->setDefaultTextColor(Qt::black);

//     QPointF position(this->width() - 250, this->height() - 150);
//     QGraphicsRectItem *textBox = createTextBox(textItem, position);
//     QGraphicsLineItem *line = createLine(entity, textItem);
//     QGraphicsTextItem *titleTextItem = createTitleTextItem(firstLine, position);
//     QGraphicsTextItem *closeIcon = createCloseIcon(position);

//     entityToTextItems[entity] = textItem;
//     entityToTextBoxs[entity] = textBox;
//     entityToLines[entity] = line;
//     entityToTitleTextItems[entity] = titleTextItem;

//     // 连接信号和槽，处理交互事件
//     connect(textItem, &QGraphicsTextItem::hoverEnterEvent, [this, entity](QGraphicsSceneHoverEvent *event) {
//         // 处理鼠标悬停事件
//     });
//     connect(textItem, &QGraphicsTextItem::hoverLeaveEvent, [this, entity](QGraphicsSceneHoverEvent *event) {
//         // 处理鼠标离开事件
//     });
//     connect(textItem, &QGraphicsTextItem::mousePressEvent, [this, entity](QGraphicsSceneMouseEvent *event) {
//         // 处理鼠标点击事件
//     });

//     scene->addItem(textItem);
//     scene->addItem(textBox);
//     scene->addItem(line);
//     scene->addItem(titleTextItem);
//     scene->addItem(closeIcon);
// }

// // 删除指定实体的文本标注
// void TextDisplayManager::closeTextActor(CEntity *entity) {
//     if (entityToTextItems.contains(entity)) {
//         QGraphicsTextItem *textItem = entityToTextItems.take(entity);
//         QGraphicsRectItem *textBox = entityToTextBoxs.take(entity);
//         QGraphicsLineItem *line = entityToLines.take(entity);
//         QGraphicsTextItem *titleTextItem = entityToTitleTextItems.take(entity);

//         scene->removeItem(textItem);
//         scene->removeItem(textBox);
//         scene->removeItem(line);
//         scene->removeItem(titleTextItem);

//         delete textItem;
//         delete textBox;
//         delete line;
//         delete titleTextItem;
//     }
// }

// // 删除所有文本标注
// void TextDisplayManager::closeAllText() {
//     for (auto it = entityToTextItems.begin(); it != entityToTextItems.end(); ++it) {
//         QGraphicsTextItem *textItem = it.value();
//         QGraphicsRectItem *textBox = entityToTextBoxs.value(it.key());
//         QGraphicsLineItem *line = entityToLines.value(it.key());
//         QGraphicsTextItem *titleTextItem = entityToTitleTextItems.value(it.key());

//         scene->removeItem(textItem);
//         scene->removeItem(textBox);
//         scene->removeItem(line);
//         scene->removeItem(titleTextItem);

//         delete textItem;
//         delete textBox;
//         delete line;
//         delete titleTextItem;
//     }

//     entityToTextItems.clear();
//     entityToTextBoxs.clear();
//     entityToLines.clear();
//     entityToTitleTextItems.clear();
// }

// // 更新文本标注的位置
// void TextDisplayManager::updateTextPosition(CEntity *entity, const QPointF &position) {
//     if (entityToTextItems.contains(entity)) {
//         QGraphicsTextItem *textItem = entityToTextItems[entity];
//         textItem->setPos(position);

//         QGraphicsRectItem *textBox = entityToTextBoxs[entity];
//         textBox->setPos(position);

//         QGraphicsLineItem *line = entityToLines[entity];
//         updateLine(entity, textItem);

//         QGraphicsTextItem *titleTextItem = entityToTitleTextItems[entity];
//         titleTextItem->setPos(position.x(), position.y() + textItem->boundingRect().height());

//         QGraphicsTextItem *closeIcon = entityToIcons.value(entity);
//         closeIcon->setPos(position.x() + textItem->boundingRect().width() + 20, position.y() + textItem->boundingRect().height() + 20);
//     }
// }

// // 创建标题文本项
// QGraphicsTextItem *TextDisplayManager::createTitleTextItem(const QString &text, const QPointF &position) {
//     QGraphicsTextItem *titleTextItem = scene->addText(text);
//     titleTextItem->setFont(QFont("Times", 18));
//     titleTextItem->setDefaultTextColor(Qt::black);
//     titleTextItem->setPos(position);
//     return titleTextItem;
// }

// // 创建文本框
// QGraphicsRectItem *TextDisplayManager::createTextBox(QGraphicsTextItem *textItem, const QPointF &position) {
//     QRectF rect = textItem->boundingRect();
//     rect.setWidth(rect.width() + 20);
//     rect.setHeight(rect.height() + 40);
//     QGraphicsRectItem *textBox = scene->addRect(rect.translated(position));
//     textBox->setPen(QPen(Qt::black));
//     textBox->setBrush(QBrush(Qt::gray));
//     return textBox;
// }

// // 创建指向线段
// QGraphicsLineItem *TextDisplayManager::createLine(CEntity *entity, QGraphicsTextItem *textItem) {
//     QPointF endPoint = entity->getEntityPosition(); // 假设 CEntity 有一个方法来获取位置
//     QPointF start = textItem->pos() + QPointF(textItem->boundingRect().width() / 2, textItem->boundingRect().height());
//     QGraphicsLineItem *line = scene->addLine(QLineF(start, endPoint));
//     line->setPen(QPen(Qt::black));
//     return line;
// }

// // 创建关闭图标文本项
// QGraphicsTextItem *TextDisplayManager::createCloseIcon(const QPointF &position) {
//     QGraphicsTextItem *closeIcon = scene->addText("X");
//     closeIcon->setFont(QFont("Arial", 12));
//     closeIcon->setDefaultTextColor(Qt::red);
//     closeIcon->setPos(position);
//     return closeIcon;
// }

// // 更新指向线段的位置
// void TextDisplayManager::updateLine(CEntity *entity, QGraphicsTextItem *textItem) {
//     QPointF endPoint = entity->getEntityPosition(); // 假设 CEntity 有一个方法来获取位置
//     QPointF start = textItem->pos() + QPointF(textItem->boundingRect().width() / 2, textItem->boundingRect().height());
//     QGraphicsLineItem *line = entityToLines.value(entity);
//     line->setLine(QLineF(start, endPoint));
// }
