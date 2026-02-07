#include "nesting_gui.h"
#include "sheet.h"
#include "algorithm.h"

// GUI 层安全转换：避免 CGAL::Uncertain_conversion_exception 刷屏
namespace {
    inline double gui_safe_to_double(const nesting::geo::FT& v) {
        auto I = CGAL::to_interval(v);
        double a = I.first;
        double b = I.second;
        if (!std::isfinite(a) || !std::isfinite(b)) {
            return 0.0;
        }
        return 0.5 * (a + b);
    }
}

nesting_gui::nesting_gui(QWidget* parent) : QMainWindow(parent) {
    ui.setupUi(this);
    // Part1 设置各表格自动拉伸
    ui.partsTable->horizontalHeader()->setSectionResizeMode(
        QHeaderView::ResizeToContents);
    ui.sheetsTable->horizontalHeader()->setSectionResizeMode(
        QHeaderView::ResizeToContents);
    ui.partsTable->setTargetColumn(2);
    ui.sheetsTable->setTargetColumn(2);
    ui.resultTable->horizontalHeader()->setSectionResizeMode(
        QHeaderView::Stretch);

    // Ensure sheets table has columns for storing additional parameters and
    // set human-readable header labels for them so numeric default headers
    // ("4", "5") are replaced with meaningful names.
    while (ui.sheetsTable->columnCount() < 6)
        ui.sheetsTable->insertColumn(ui.sheetsTable->columnCount());
    // Column 3 stores the sheet type flag
    QTableWidgetItem* headerType = new QTableWidgetItem();
    headerType->setText("Type");
    ui.sheetsTable->setHorizontalHeaderItem(3, headerType);
    // Column 4 stores segments (used for circle sheets)
    QTableWidgetItem* headerSegments = new QTableWidgetItem();
    headerSegments->setText("Segments");
    ui.sheetsTable->setHorizontalHeaderItem(4, headerSegments);
    // Column 5 stores Diameter for circle sheets
    QTableWidgetItem* headerDiameter = new QTableWidgetItem();
    headerDiameter->setText("Diameter");
    ui.sheetsTable->setHorizontalHeaderItem(5, headerDiameter);

    // helper to update visibility of columns for a given row based on type
    auto updateColumnsForRow = [this](int row) {
        if (row < 0 || row >= ui.sheetsTable->rowCount()) return;
        int colType = 3;
        int colPreview = 2;
        int colLength = 0;
        int colWidth = 1;
        int colDiameter = 5;
        int typeFlag = -1;
        auto typeItem = ui.sheetsTable->item(row, colType);
        if (typeItem) {
            typeFlag = typeItem->data(Qt::UserRole).toInt();
        } else {
            auto prev = ui.sheetsTable->item(row, colPreview);
            if (prev) typeFlag = prev->data(Qt::UserRole).toInt();
        }
        bool isCircle = (typeFlag == 1);
        ui.sheetsTable->setColumnHidden(colLength, isCircle);
        ui.sheetsTable->setColumnHidden(colWidth, isCircle);
        ui.sheetsTable->setColumnHidden(colDiameter, !isCircle);
    };

    // If user edits the Type cell later, reflect column visibility change
    connect(ui.sheetsTable, &QTableWidget::itemChanged, this, [this, updateColumnsForRow](QTableWidgetItem* item){
        if (!item) return;
        // only respond when type or preview column changes
        if (item->column() == 3 || item->column() == 2) {
            updateColumnsForRow(item->row());
        }
    });

    // Part2 设置QToolButton的action
    ui.partsOpenCAD->setDefaultAction(ui.actionOpen_DXF_DWG_File);
    ui.partsOpenCSV->setDefaultAction(ui.actionOpen_CSV_File);
    ui.partsOpenSVG->setDefaultAction(ui.actionOpen_SVG_File);
    ui.CreateSheetButton->setDefaultAction(ui.actionCreate_Sheet);
    ui.ExportTXTButton->setDefaultAction(ui.actionTo_TXT_File);
    ui.ExportSVGButton->setDefaultAction(ui.actionTo_SVG_File);
    ui.ExportDXFButton->setDefaultAction(ui.actionTo_DXF_File);
    // Part3 设置Nesting Tab中各控件的初始状态
    ui.fixRun->setChecked(true);
    ui.StopButton->setDisabled(true);
    ui.progressBar->setValue(0);
    // Part4 设置图表对象

    // 坐标轴
    axisX = new QValueAxis();
    axisY = new QValueAxis();
    axisX->setRange(0, 60);
    axisX->setTickCount(7);
    axisY->setRange(0, 1);
    axisX->setTitleText("Time(s)");
    axisY->setTitleText("Util(%)");
    // 值
    series = new QLineSeries();
    series->setName("Util(%)");
    series->useOpenGL();
    // series->setPointsVisible(true);
    //  图像
    chart = new QChart();
    chart->addAxis(axisX, Qt::AlignBottom);
    chart->addAxis(axisY, Qt::AlignLeft);
    chart->addSeries(series);
    // chart->setAnimationOptions(QChart::SeriesAnimations);
    chart->legend()->hide();
    series->attachAxis(axisX);
    series->attachAxis(axisY);

    ui.resultChart->setRenderHint(QPainter::Antialiasing);
    ui.resultChart->setChart(chart);
    ui.resultChart->setRubberBand(QChartView::VerticalRubberBand);
    // 滚动条
    ui.ChartScrollBar->setRange(0, 30);
    connect(ui.ChartScrollBar, &QScrollBar::valueChanged, [&](int value) {
        // 计算偏移量
        static bool scrolling = false;
        static int previous = 0;
        // 在水平方向上滚动
        if (!scrolling) {
            scrolling = true;
            ui.resultChart->chart()->axisX()->setRange(value, value + 60);
            previous = value;
            scrolling = false;
        }
        });
    // Part5 设置CurrentLayout
    // 设置dockwidget只在tabindex=2时显现
    connect(ui.tabWidget, &QTabWidget::currentChanged, ui.dockWidget,
        [&](int index) {
            if (index != 2) {
                ui.dockWidget->hide();
            }
            else {
                ui.dockWidget->show();
            }
        });
    ui.dockWidget->setWidget(ui.openGLWidget);
    ui.dockWidget->hide();
    this->addDockWidget(Qt::BottomDockWidgetArea, ui.dockWidget);
    this->resizeDocks({ ui.dockWidget }, { (int)(this->height() * 0.5) },
        Qt::Vertical);
    // Part7 设置时钟
    timer = new QTimer(this);

    connect(timer, &QTimer::timeout, [&]() {
        // 处理progressBar
        auto max = ui.progressBar->maximum();
        auto min = ui.progressBar->minimum();
        if (max != 0 || min != 0) {
            if (time == max) {
                time = 0;
            }
            ui.progressBar->setValue(time);
        }
        if (ui.resultTable->rowCount() > 0) {
            series->append(
                time, ui.resultTable->item(0, 1)->data(Qt::DisplayRole).toReal());
        }
        else {
            series->append(time, 0);
        }
        ui.ChartScrollBar->setMaximum(ui.ChartScrollBar->maximum() + 1);
        ++time;
        });
}

nesting_gui::~nesting_gui() {
    delete chart;
    delete series;
    delete axisX;
    delete axisY;
    delete timer;
}

void nesting_gui::createSheet() {
    auto rowIndex = ui.sheetsTable->rowCount();
    if (rowIndex >= 1) {
        QMessageBox::warning(this, "Warning",
            "Currently, only one sheet is allowed to be used.");
        return;
    }
    ui.tabWidget->setCurrentIndex(1);
    QDialog dialog(this);
    UICreateSheetDialog.setupUi(&dialog);
    // 默认根据类型切换可见性
    auto typeCombo = UICreateSheetDialog.type;
    auto labelDiameter = UICreateSheetDialog.label_diameter;
    auto diameterSpin = UICreateSheetDialog.diameter;
    auto labelSegments = UICreateSheetDialog.label_segments;
    auto segmentsSpin = UICreateSheetDialog.segments;
    auto labelLength = UICreateSheetDialog.label_length;
    auto labelWidth = UICreateSheetDialog.label_width;
    auto widthSpin = UICreateSheetDialog.width;
    auto heightSpin = UICreateSheetDialog.height;
    auto labelQuantity = UICreateSheetDialog.label_3;
    auto quantitySpin = UICreateSheetDialog.quantity;
    labelQuantity->setVisible(false);
    quantitySpin->setVisible(false);
    auto updateVisibility = [&]() {
        // 只支持圆形板材：强制使用 Circle 类型
        bool isCircle = true;
        labelDiameter->setVisible(isCircle);
        diameterSpin->setVisible(isCircle);
        labelSegments->setVisible(isCircle);
        segmentsSpin->setVisible(isCircle);
        labelLength->setVisible(!isCircle);
        labelWidth->setVisible(!isCircle);
        widthSpin->setVisible(!isCircle);
        heightSpin->setVisible(!isCircle);
    };
    // 强制为圆形，并禁用类型选择（彻底移除矩形选项）
    typeCombo->setCurrentIndex(1);
    typeCombo->setDisabled(true);
    updateVisibility();
    connect(typeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), [&](int){ updateVisibility(); });

    if (QDialog::Accepted == dialog.exec()) {
        ui.sheetsTable->insertRow(rowIndex);
        // Add hidden columns to store shape type, segments and diameter
        while (ui.sheetsTable->columnCount() < 6) ui.sheetsTable->insertColumn(ui.sheetsTable->columnCount());
        int colType = 3;
        int colSegments = 4;
        int colW = 0;
        int colH = 1;
        int colPreview = 2;
        int colDiameter = 5;
        QPolygonF p;
        double w = 0, h = 0;
        int typeFlag = typeCombo->currentIndex();
        if (typeFlag == 1) {
            // Circle
            double d = diameterSpin->value();
            // store diameter in dedicated column, leave length/width blank
            auto iDia = new QTableWidgetItem(); iDia->setData(Qt::DisplayRole, d);
            ui.sheetsTable->setItem(rowIndex, colDiameter, iDia);
            // keep length/width cells empty for clarity
            ui.sheetsTable->setItem(rowIndex, colW, new QTableWidgetItem());
            ui.sheetsTable->setItem(rowIndex, colH, new QTableWidgetItem());

            const int segs = UICreateSheetDialog.segments->value();
            constexpr double kPi = 3.14159265358979323846;
            for (int i = 0; i < segs; ++i) {
                double theta = (2.0 * kPi * i) / segs;
                double x = (d / 2.0) + (d / 2.0) * std::cos(theta);
                double y = (d / 2.0) + (d / 2.0) * std::sin(theta);
                p.append(QPointF(x, y));
            }
            auto i4 = new QTableWidgetItem();
            i4->setData(Qt::DisplayRole, p);
            i4->setData(Qt::UserRole, typeFlag);
            ui.sheetsTable->setItem(rowIndex, colPreview, i4);
            auto iType = new QTableWidgetItem();
            // display human-readable type instead of numeric
            iType->setData(Qt::DisplayRole, QString("Circle"));
            iType->setData(Qt::UserRole, typeFlag);
            ui.sheetsTable->setItem(rowIndex, colType, iType);
            // store segments value
            auto segItem = new QTableWidgetItem();
            segItem->setData(Qt::DisplayRole, UICreateSheetDialog.segments->value());
            ui.sheetsTable->setItem(rowIndex, colSegments, segItem);
        } else {
            // Rectangle
            w = widthSpin->value();
            h = heightSpin->value();
            auto i1 = new QTableWidgetItem(); i1->setData(Qt::DisplayRole, w);
            ui.sheetsTable->setItem(rowIndex, colW, i1);
            auto i2 = new QTableWidgetItem(); i2->setData(Qt::DisplayRole, h);
            ui.sheetsTable->setItem(rowIndex, colH, i2);
            p = QPolygonF({ QPointF(0, 0), QPointF(w, 0), QPointF(w, h), QPointF(0, h) });
            auto i4 = new QTableWidgetItem();
            i4->setData(Qt::DisplayRole, p);
            i4->setData(Qt::UserRole, typeFlag);
            ui.sheetsTable->setItem(rowIndex, colPreview, i4);
            auto iType = new QTableWidgetItem();
            // display human-readable type instead of numeric
            iType->setData(Qt::DisplayRole, QString("Rectangle"));
            iType->setData(Qt::UserRole, typeFlag);
            ui.sheetsTable->setItem(rowIndex, colType, iType);
            // for rectangle, set segments and diameter to 0/blank
            auto segItem = new QTableWidgetItem();
            segItem->setData(Qt::DisplayRole, 0);
            ui.sheetsTable->setItem(rowIndex, colSegments, segItem);
            ui.sheetsTable->setItem(rowIndex, colDiameter, new QTableWidgetItem());
        }
        // ensure columns visibility reflects stored type
        // reuse lambda defined in constructor scope
        // find lambda by creating a small local copy
        auto updateColumnsForRowLocal = [this](int row) {
            if (row < 0 || row >= ui.sheetsTable->rowCount()) return;
            int colType = 3;
            int colPreview = 2;
            int colLength = 0;
            int colWidth = 1;
            int colDiameter = 5;
            int typeFlag = -1;
            auto typeItem = ui.sheetsTable->item(row, colType);
            if (typeItem) {
                typeFlag = typeItem->data(Qt::UserRole).toInt();
            } else {
                auto prev = ui.sheetsTable->item(row, colPreview);
                if (prev) typeFlag = prev->data(Qt::UserRole).toInt();
            }
            bool isCircle = (typeFlag == 1);
            ui.sheetsTable->setColumnHidden(colLength, isCircle);
            ui.sheetsTable->setColumnHidden(colWidth, isCircle);
            ui.sheetsTable->setColumnHidden(colDiameter, !isCircle);
        };
        updateColumnsForRowLocal(rowIndex);
    }
}

void nesting_gui::openCSV() {
    ui.tabWidget->setCurrentIndex(0);
    QString filePath =
        QFileDialog::getOpenFileName(this, "Open CSV File", ".", "CSV (*.csv)");
    if (filePath.isEmpty()) {
        return;
    }
    std::vector<nesting::geo::Polygon_with_holes_2> pgns;
    std::vector<std::uint32_t> rots;
    std::vector<std::uint32_t> quat;
    try {
        nesting::read_from_csv(filePath.toStdString(), pgns, rots, quat);
    }
    catch (const std::runtime_error& e) {
        QMessageBox::warning(this, "Open CSV File", e.what());
    }
    storeData(pgns, rots, quat);
}

void nesting_gui::openCAD() {
    ui.tabWidget->setCurrentIndex(0);
    QStringList filePaths = QFileDialog::getOpenFileNames(
        this, "Open DXF/DWG File", ".", "CAD (*.dxf *.dwg)");
    if (filePaths.isEmpty()) {
        return;
    }
    std::vector<nesting::geo::Polygon_with_holes_2> pgns;
    std::vector<std::uint32_t> rots;
    std::vector<std::uint32_t> quat;
    for (auto& qstring : filePaths) {
        try {
            nesting::read_from_cad(qstring.toStdString(), pgns, rots, quat);
        }
        catch (const std::runtime_error& e) {
            QMessageBox::warning(this, "Open CAD File", e.what());
        }
    }
    storeData(pgns, rots, quat);
}

void nesting_gui::openSVG() {
    ui.tabWidget->setCurrentIndex(0);
    QStringList filePaths =
        QFileDialog::getOpenFileNames(this, "Open SVG File", ".", "SVG (*.svg)");
    if (filePaths.isEmpty()) {
        return;
    }
    std::vector<nesting::geo::Polygon_with_holes_2> pgns;
    std::vector<std::uint32_t> rots;
    std::vector<std::uint32_t> quat;
    for (auto& qstring : filePaths) {
        try {
            nesting::read_from_svg(qstring.toStdString(), pgns, rots, quat);
        }
        catch (const std::runtime_error& e) {
            QMessageBox::warning(this, "Open SVG File", e.what());
        }
    }
    storeData(pgns, rots, quat);
}

void nesting_gui::toTXT() {
    if (ui.resultTable->rowCount() == 0) {
        QMessageBox::warning(this, "Warning", "Solutions do not exist");
        return;
    }
    QString filePath = QFileDialog::getSaveFileName(this, "Save File", ".",
        "Text files (*.txt)");
    if (!filePath.isEmpty()) {
        auto i1 = ui.resultTable->item(0, 0);
        auto i2 = ui.resultTable->item(0, 1);
        auto i3 = ui.resultTable->item(0, 2);
        auto util = i2->data(Qt::DisplayRole).value<double>();
        auto pgns = i2->data(Qt::UserRole)
            .value<std::vector<nesting::geo::Polygon_with_holes_2>>();
        // read sheet dimensions depending on type
        double sheet_length = 0;
        double sheet_width = 0;
        auto prev = ui.sheetsTable->item(0, 2);
        int typeFlag = prev ? prev->data(Qt::UserRole).toInt() : 0;
        if (typeFlag == 1) {
            // circle: diameter stored in column 5
            auto diaItem = ui.sheetsTable->item(0, 5);
            if (diaItem) sheet_width = diaItem->data(Qt::DisplayRole).value<double>();
            sheet_length = 0;
        } else {
            auto lenIt = ui.sheetsTable->item(0, 0);
            auto widIt = ui.sheetsTable->item(0, 1);
            if (lenIt) sheet_length = lenIt->data(Qt::DisplayRole).value<double>();
            if (widIt) sheet_width = widIt->data(Qt::DisplayRole).value<double>();
        }
        auto ret = nesting::write_to_txt(filePath.toStdString(), util, sheet_width,
            sheet_length, pgns);
        if (ret) {
            QMessageBox::information(this, "Success", "File saved successfully");
        }
        else {
            QMessageBox::warning(this, "Fail", "File failed to save");
        }
    }
}

void nesting_gui::toDXF() {
    if (ui.resultTable->rowCount() == 0) {
        QMessageBox::warning(this, "Warning", "Solutions do not exist");
        return;
    }
    QString filePath =
        QFileDialog::getSaveFileName(this, "Save File", ".", "DXF (*.dxf)");
    if (!filePath.isEmpty()) {
        auto i1 = ui.resultTable->item(0, 0);
        auto i2 = ui.resultTable->item(0, 1);
        auto i3 = ui.resultTable->item(0, 2);
        auto util = i2->data(Qt::DisplayRole).value<double>();
        auto pgns = i2->data(Qt::UserRole)
            .value<std::vector<nesting::geo::Polygon_with_holes_2>>();
        double sheet_length = 0;
        double sheet_width = 0;
        auto prev = ui.sheetsTable->item(0, 2);
        int typeFlag = prev ? prev->data(Qt::UserRole).toInt() : 0;
        if (typeFlag == 1) {
            auto diaItem = ui.sheetsTable->item(0, 5);
            if (diaItem) sheet_width = diaItem->data(Qt::DisplayRole).value<double>();
            sheet_length = 0;
        } else {
            auto lenIt = ui.sheetsTable->item(0, 0);
            auto widIt = ui.sheetsTable->item(0, 1);
            if (lenIt) sheet_length = lenIt->data(Qt::DisplayRole).value<double>();
            if (widIt) sheet_width = widIt->data(Qt::DisplayRole).value<double>();
        }
        auto ret = nesting::write_to_dxf(filePath.toStdString(), util, sheet_width,
            sheet_length, pgns, (typeFlag == 1));
        if (ret) {
            QMessageBox::information(this, "Success", "File saved successfully");
        }
        else {
            QMessageBox::warning(this, "Fail", "File failed to save");
        }
    }
}

void nesting_gui::toSVG() {
    if (ui.resultTable->rowCount() == 0) {
        QMessageBox::warning(this, "Warning", "Solutions do not exist");
        return;
    }
    QString filePath =
        QFileDialog::getSaveFileName(this, "Save File", ".", "SVG (*.svg)");
    if (!filePath.isEmpty()) {
        auto i1 = ui.resultTable->item(0, 0);
        auto i2 = ui.resultTable->item(0, 1);
        auto i3 = ui.resultTable->item(0, 2);
        auto util = i2->data(Qt::DisplayRole).value<double>();
        auto pgns = i2->data(Qt::UserRole)
            .value<std::vector<nesting::geo::Polygon_with_holes_2>>();
        double sheet_length = 0;
        double sheet_width = 0;
        auto prev = ui.sheetsTable->item(0, 2);
        int typeFlag = prev ? prev->data(Qt::UserRole).toInt() : 0;
        if (typeFlag == 1) {
            auto diaItem = ui.sheetsTable->item(0, 5);
            if (diaItem) sheet_width = diaItem->data(Qt::DisplayRole).value<double>();
            sheet_length = 0;
        } else {
            auto lenIt = ui.sheetsTable->item(0, 0);
            auto widIt = ui.sheetsTable->item(0, 1);
            if (lenIt) sheet_length = lenIt->data(Qt::DisplayRole).value<double>();
            if (widIt) sheet_width = widIt->data(Qt::DisplayRole).value<double>();
        }
        auto ret = nesting::write_to_svg(filePath.toStdString(), util, sheet_width,
            sheet_length, pgns);
        if (ret) {
            QMessageBox::information(this, "Success", "File saved successfully");
        }
        else {
            QMessageBox::warning(this, "Fail", "File failed to save");
        }
    }
}

void nesting_gui::start() {
    // 清空上次的ui
    qDebug() << "start";
    qDebug() << "[DEBUG] ========== 开始排料 ==========";
    ui.progressBar->setRange(0, 100);
    ui.progressBar->setValue(0);
    series->clear();
    ui.resultTable->clearContents();
    ui.resultTable->setRowCount(0);
    auto parts_count = ui.partsTable->rowCount();
    if (parts_count == 0) {
        QMessageBox::warning(this, "Warning", "Parts do not exsit");
        return;
    }
    auto sheets_count = ui.sheetsTable->rowCount();
    if (sheets_count == 0) {
        QMessageBox::warning(this, "Warning", "Sheet does not exsit");
        return;
    }
    // 获取part
    std::vector<uint32_t> allowed_rotations;
    std::vector<uint32_t> quantity;
    std::vector<nesting::geo::Polygon_with_holes_2> polygons;
    for (size_t i = 0; i < parts_count; i++) {
        QTableWidgetItem* i0 = ui.partsTable->item(i, 0);
        QTableWidgetItem* i1 = ui.partsTable->item(i, 1);
        QTableWidgetItem* i2 = ui.partsTable->item(i, 2);
        auto allowed_rotation = i0->data(Qt::DisplayRole).value<uint32_t>();
        auto qua = i1->data(Qt::DisplayRole).value<uint32_t>();
        auto pgn =
            i2->data(Qt::UserRole).value<nesting::geo::Polygon_with_holes_2>();
        allowed_rotations.push_back(allowed_rotation);
        quantity.push_back(qua);
        polygons.push_back(pgn);
    }
    // 获取sheet
    QTableWidgetItem* i0 = ui.sheetsTable->item(0, 0);
    QTableWidgetItem* i1 = ui.sheetsTable->item(0, 1);
    QTableWidgetItem* iPreview = ui.sheetsTable->item(0, 2);
    double sheet_width = 0;
    double sheet_height = 0;
    int typeFlag = iPreview ? iPreview->data(Qt::UserRole).value<int>() : 0;
    if (typeFlag == 1) {
        // Circle: diameter stored in column 5
        auto diaItem = ui.sheetsTable->item(0, 5);
        if (diaItem) sheet_width = diaItem->data(Qt::DisplayRole).value<double>();
        sheet_height = 0;
        // set GL widget accordingly
        ui.openGLWidget->set_sheet(sheet_width, 0);
    } else {
        if (i0) sheet_width = i0->data(Qt::DisplayRole).value<double>();
        if (i1) sheet_height = i1->data(Qt::DisplayRole).value<double>();
        ui.openGLWidget->set_sheet(sheet_width, sheet_height);
    }
    // 时间参数
    int seconds = 24 * 3600;
    if (ui.fixRun->isChecked()) {
        auto time = ui.timeEdit->time();
        auto h = time.hour();
        auto m = time.minute();
        auto s = time.second();
        seconds = (h * 3600 + m * 60 + s);
        ui.progressBar->setRange(0, seconds);
    }
    else {
        ui.progressBar->setRange(0, 0);
    }
    // 其他参数
    auto need_simplify = ui.needSimplify->isChecked();
    auto fast_mode = ui.fastModeCheckBox ? ui.fastModeCheckBox->isChecked() : false;
    auto use_clipper = ui.useClipperCheckBox ? ui.useClipperCheckBox->isChecked() : false;
    auto part_offset = ui.partSpacingSpinBox->value();
    auto top_offset = ui.TopSpinBox->value();
    auto bottom_offset = ui.BottomSpinBox->value();
    auto left_offset = ui.LeftSpinBox->value();
    auto right_offset = ui.RightSpinBox->value();
    // Read segments value only if sheet is circle
    int segments = 128;
    if (typeFlag == 1) {
        if (ui.sheetsTable->columnCount() > 4) {
            auto segItem = ui.sheetsTable->item(0, 4);
            if (segItem) segments = segItem->data(Qt::DisplayRole).toInt();
        }
    }
    // ui更新
    ui.ChartScrollBar->setRange(0, 30);
    ui.ChartScrollBar->setValue(0);
    ui.resultChart->chart()->axisX()->setRange(0, 60);
    ui.startButton->setDisabled(true);
    ui.StopButton->setEnabled(true);
    ui.timeEdit->setDisabled(true);
    ui.partSpacingSpinBox->setDisabled(true);
    ui.TopSpinBox->setDisabled(true);
    ui.LeftSpinBox->setDisabled(true);
    ui.BottomSpinBox->setDisabled(true);
    ui.RightSpinBox->setDisabled(true);
    ui.fixRun->setDisabled(true);
    ui.needSimplify->setDisabled(true);
    if (ui.fastModeCheckBox) {
        ui.fastModeCheckBox->setDisabled(true);
    }
    if (ui.useClipperCheckBox) {
        ui.useClipperCheckBox->setDisabled(true);
    }
    // 开启时钟
    time = 0;
    timer->start(1000);
    startWork(need_simplify, top_offset, left_offset, bottom_offset, right_offset,
        part_offset, sheet_width, sheet_height, seconds, polygons,
        allowed_rotations, quantity, segments, fast_mode, use_clipper);
}

void nesting_gui::stop() {
    qDebug() << "stop START";
    // 关闭定时任务
    if (timer->isActive()) {
        timer->stop();
    }
    // 强制停止线程（恢复为原有逻辑：使用等待对话框 + 事件循环）
    if (worker != nullptr && worker->isRunning()) {
        waitDialog = new QDialog(this);
        UIWaitDialog.setupUi(waitDialog);
        // 先请求线程优雅退出
        worker->quit();
        worker->requestQuit();
        // 等待对话框不允许用户自行关闭
        waitDialog->setWindowFlag(Qt::WindowCloseButtonHint, false);
        waitDialog->open();
        // 使用本地事件循环等待线程结束
        QEventLoop loop;
        connect(worker, &QThread::finished, &loop, &QEventLoop::quit);
        loop.exec();
    }
    qDebug() << "stop END";
}

void nesting_gui::handleProgress(Solution solu,
    const std::vector<nesting::Item>& sim,
    const std::vector<nesting::Item>& ori) {
    qDebug() << "handleProgress START";
    qDebug() << solu.length;
    qDebug() << solu.utilization;
    // Part1 更新result table
    auto rowIndex = ui.resultTable->rowCount();
    ui.resultTable->insertRow(0);

    auto i2 = new QTableWidgetItem();
    auto i3 = new QTableWidgetItem();
    i3->setData(Qt::DisplayRole, solu.time);
    ui.resultTable->setItem(0, 2, i3);
    auto part_offset = ui.partSpacingSpinBox->value();
    auto bottom_offset = ui.BottomSpinBox->value();
    auto left_offset = ui.LeftSpinBox->value();
    auto i1 = new QTableWidgetItem();
    auto pgns = nesting::postprocess(solu.solution, part_offset, left_offset, bottom_offset, sim, ori);

    // 使用“当前已经排布在圆内的刀头”面积来计算利用率：
    // 每次回调，根据当前解中在圆内的刀头面积 / 圆面积，得到一个时序利用率，连成曲线。
    double diameter = solu.length;
    double radius = diameter / 2.0;
    double cx = radius;
    double cy = radius;
    double radius_sq = radius * radius;

    double parts_area = 0.0;
    for (const auto& pwh : pgns) {
        // 先判断该刀头是否在圆内（或至少有顶点在圆内），过滤掉还没排进来的刀头
        bool in_circle = false;
        const auto& outer = pwh.outer_boundary();
        for (auto v = outer.vertices_begin(); v != outer.vertices_end(); ++v) {
            double x = gui_safe_to_double(v->x());
            double y = gui_safe_to_double(v->y());
            double dx = x - cx;
            double dy = y - cy;
            double dist_sq = dx * dx + dy * dy;
            if (dist_sq <= radius_sq + 1e-6) {
                in_circle = true;
                break;
            }
        }
        if (!in_circle) {
            continue;
        }

        try {
            parts_area += gui_safe_to_double(nesting::geo::pwh_area(pwh));
        } catch (...) {
            // 忽略单个多边形的面积计算错误
        }
    }

    // 圆面积：使用 solu.length 作为直径，和核心算法 / DXF 完全一致
    double circle_area = nesting::Sheet::kPi * radius * radius;
    double util = 0.0;
    if (circle_area > 0.0) {
        util = parts_area / circle_area;
        if (util < 0.0) util = 0.0;
        if (util > 1.0) util = 1.0;
    }

    i2->setData(Qt::DisplayRole, util);
    ui.resultTable->setItem(0, 1, i2);
    QVariant var1;
    var1.setValue(pgns);
    i2->setData(Qt::UserRole, var1);
    auto size = pgns.size();
    QList<QPolygonF> display_pgns;
    for (size_t i = 0; i < size; i++) {
        QPolygonF polygon;
        auto& pwh = pgns[i];
        auto& outer = pwh.outer_boundary();
        for (auto v = outer.vertices_begin(); v != outer.vertices_end(); v++) {
            polygon.append(QPointF(gui_safe_to_double(v->x()), gui_safe_to_double(v->y())));
        }
        display_pgns.append(polygon);
    }
    QVariant var2;
    var2.setValue(display_pgns);
    i1->setData(Qt::UserRole, var2);
    i1->setData(Qt::DisplayRole, solu.length);
    ui.resultTable->setItem(0, 0, i1);
    ui.resultTable->setCurrentCell(0, 0);
    // Part2 更新result chart
    series->append(solu.time, util);
    qDebug() << "handleProgress END";
}

void nesting_gui::startWork(
    const bool need_simplify,
    const double top_offset,
    const double left_offset,
    const double bottom_offset,
    const double right_offset,
    const double part_offset,
    const double sheet_width,
    const double sheet_height,
    const size_t max_time,
    const std::vector<nesting::geo::Polygon_with_holes_2>& polygons,
    const std::vector<uint32_t>& items_rotations,
    const std::vector<uint32_t>& items_quantity,
    const int circle_segments,
    const bool fast_mode,
    const bool use_clipper) {
    // Part6 设置线程
    qDebug() << "start work START";
    worker = new Worker(this);
    connect(worker, &Worker::resultReady, this, &nesting_gui::handleProgress);
    connect(worker, &Worker::finished, this, &nesting_gui::handleFinished);
    connect(worker, &Worker::sendMessage, this, &nesting_gui::handleMessage);
    worker->set(need_simplify, top_offset, left_offset, bottom_offset,
        right_offset, part_offset, sheet_width, sheet_height, max_time,
        polygons, items_rotations, items_quantity, circle_segments, fast_mode, use_clipper);
    worker->start();
    qDebug() << "start work END";
}

void nesting_gui::storeData(
    std::vector<nesting::geo::Polygon_with_holes_2> pgns,
    std::vector<std::uint32_t> rots,
    std::vector<std::uint32_t> quat) {
    auto numRows = pgns.size();
    ui.partsTable->setRowCount(numRows);
    for (size_t i = 0; i < numRows; ++i) {
        auto i1 = new QTableWidgetItem();
        i1->setData(Qt::DisplayRole, rots[i]);
        ui.partsTable->setItem(i, 0, i1);
        auto i2 = new QTableWidgetItem();
        i2->setData(Qt::DisplayRole, quat[i]);
        ui.partsTable->setItem(i, 1, i2);
        auto i3 = new QTableWidgetItem();
        qreal px = 0;
        qreal py = 0;
        QPolygonF pgn;
        for (auto& v : pgns[i].outer_boundary()) {
            px = CGAL::to_double(v.x());
            py = CGAL::to_double(v.y());
            pgn.append(QPointF(px, py));
        }
        i3->setData(Qt::DisplayRole, pgn);
        QVariant var;
        var.setValue(pgns[i]);
        i3->setData(Qt::UserRole, var);
        ui.partsTable->setItem(i, 2, i3);
    }
}

void nesting_gui::handleFinished() {
    qDebug() << "handleFinished()";
    //线程对象应该会自动释放
    if (worker != nullptr) {
        qDebug() << worker->isFinished();
        delete worker;
        worker = nullptr;
    }
    if (waitDialog != nullptr) {
        waitDialog->close();
        delete waitDialog;
        waitDialog = nullptr;
    }
    ui.StopButton->click();
    QMessageBox::information(this, "Info", "Engine thread exited");
    qDebug() << "handleFinished() END";
}

void nesting_gui::handleMessage(const QString& s) {
    qDebug() << "handleMessage(): " << s;
    QMessageBox::critical(this, "Error", s);
}
