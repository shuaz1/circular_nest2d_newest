#include "layout_widget.h"

void LayoutWidget::initialize_transform() {
    globalTransform.reset();
    auto r = rect();
    // guard against zero sheet size
    qreal sheetW = sheet.width() > 0 ? sheet.width() : 1.0;
    qreal sheetH = sheet.height() > 0 ? sheet.height() : 1.0;
    qreal scaleX = (r.width() - 5) / sheetW;
    qreal scaleY = (r.height() - 5) / sheetH;
    qreal scale = qMin(scaleX, scaleY);
    globalTransform.translate(r.center().rx() - sheet.center().rx() * scale,
        r.center().ry() - sheet.center().ry() * scale);
    globalTransform.scale(scale, scale);
}

void LayoutWidget::set_sheet(qreal w, qreal h) {
    // 圆形板材用h==0作为标志
    if (h == 0) {
        sheet = QRectF(0, 0, w, w);
        sheetIsCircle = true;
    } else {
        sheet = QRectF(0, 0, w, h);
        sheetIsCircle = false;
    }
}

void LayoutWidget::initializeGL() {
    initializeOpenGLFunctions();
    initialize_transform();
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
}

void LayoutWidget::paintEvent(QPaintEvent* event) {
    QPainter painter;
    painter.begin(this);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.fillRect(rect(), Qt::white);
    painter.save();

    qreal scaledPenWidth = 2.0 / globalTransform.m11();
    QPen scaledPen = painter.pen();
    if (scaledPenWidth == 0) {
        scaledPenWidth = 1;
    }
    scaledPen.setWidthF(scaledPenWidth);

    // 设置sheet边界线更宽
    QPen sheetPen = scaledPen;
    sheetPen.setWidthF(sheetPen.widthF() * 1.5);

    // 绘制sheet
    painter.setTransform(globalTransform);
    painter.setPen(sheetPen);
    // Only render ellipse when sheetIsCircle is explicitly set
    if (sheetIsCircle && sheet.width() > 0 && sheet.height() > 0) {
        painter.drawEllipse(QRectF(0, 0, sheet.width(), sheet.height()));
    } else {
        painter.drawRect(sheet);
    }

    // 绘制多边形
    QPen polygonPen = scaledPen;  // 使用相同的线宽
    painter.setPen(polygonPen);
    painter.setBrush(Qt::gray);
    for (auto& p : layout) {
        painter.drawPolygon(p);
    }

    painter.restore();
    painter.end();
}

void LayoutWidget::wheelEvent(QWheelEvent* event) {
    int delta = event->angleDelta().y();
    QPointF mousePos = event->position();
    auto mapPt = globalTransform.inverted().map(mousePos);
    qreal mapX = mapPt.x();
    qreal mapY = mapPt.y();
    qreal scaleFactor = 1.1;
    if (delta < 0) {
        scaleFactor = 1.0 / scaleFactor;
    }
    globalTransform.translate(mapX, mapY);
    globalTransform.scale(scaleFactor, scaleFactor);
    globalTransform.translate(-mapX, -mapY);
    update();
}

void LayoutWidget::mousePressEvent(QMouseEvent* event) {
    if (event->button() == Qt::LeftButton) {
        m_lastPos = event->pos();
    }
}

void LayoutWidget::mouseMoveEvent(QMouseEvent* event) {
    if (event->buttons() & Qt::LeftButton) {
        QPointF delta = event->pos() - m_lastPos;
        globalTransform.translate(delta.x() / globalTransform.m11(),
            delta.y() / globalTransform.m22());
        update();
        m_lastPos = event->pos();
    }
}

void LayoutWidget::layoutUpdate(QTableWidgetItem* n, QTableWidgetItem* o) {
    qDebug() << "layoutUpdate START";
    if (n != nullptr) {
        auto i = n->tableWidget()->item(n->row(), 0);
        auto v = i->data(Qt::UserRole).value<QList<QPolygonF>>();
        auto len = i->data(Qt::DisplayRole).value<qreal>();
        this->layout = v;
        this->length = len;
        initialize_transform();
        update();
    }
}
