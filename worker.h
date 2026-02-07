#pragma once
#include <QDebug>
#include <QThread>
#include "interface.h"

class Worker : public QThread {
    Q_OBJECT
public:
    Worker(QObject* parent = nullptr) : QThread(parent) {}
    ~Worker() {
        qDebug() << "~Worker()";
        // emit sendMessage("~Worker()");
        if (p != nullptr) {
            delete p;
        }
    }
    void requestQuit() {
        //if (p != nullptr) {
        //  p->layout.isRequestQuit = true;
        //}
        isRequestQuit = true;
    }
    void set(const bool _need_simplify,
        const double _top_offset,
        const double _left_offset,
        const double _bottom_offset,
        const double _right_offset,
        const double _part_offset,
        const double _sheet_width,
        const double _sheet_height,
        const size_t _max_time,
        const std::vector<nesting::geo::Polygon_with_holes_2>& _polygons,
        const std::vector<uint32_t>& _items_rotations,
        const std::vector<uint32_t>& _items_quantity,
        const bool _fast_mode,
        const bool _use_clipper) {
        need_simplify = _need_simplify;
        top_offset = _top_offset;
        left_offset = _left_offset;
        bottom_offset = _bottom_offset;
        right_offset = _right_offset;
        part_offset = _part_offset;
        sheet_width = _sheet_width;
        sheet_height = _sheet_height;
        max_time = _max_time;
        polygons = _polygons;
        items_rotations = _items_rotations;
        items_quantity = _items_quantity;
        fast_mode = _fast_mode;
        use_clipper = _use_clipper;
    }
    void set(const bool _need_simplify,
        const double _top_offset,
        const double _left_offset,
        const double _bottom_offset,
        const double _right_offset,
        const double _part_offset,
        const double _sheet_width,
        const double _sheet_height,
        const size_t _max_time,
        const std::vector<nesting::geo::Polygon_with_holes_2>& _polygons,
        const std::vector<uint32_t>& _items_rotations,
        const std::vector<uint32_t>& _items_quantity,
        const int _circle_segments,
        const bool _fast_mode,
        const bool _use_clipper) {
        set(_need_simplify, _top_offset, _left_offset, _bottom_offset, _right_offset,
            _part_offset, _sheet_width, _sheet_height, _max_time,
            _polygons, _items_rotations, _items_quantity, _fast_mode, _use_clipper);
        circle_segments = _circle_segments;
    }
    void run() override {
        // 在这里执行耗时的操作
        qDebug() << "CustomThread is running in thread:"
            << QThread::currentThreadId();

        doWork();
    }

signals:
    void resultReady(Solution,
        const std::vector<nesting::Item>&,
        const std::vector<nesting::Item>&);
    void sendMessage(const QString&);

private:
    volatile bool isRequestQuit{ false };
    bool need_simplify{ false };
    double top_offset{ 0 };
    double left_offset{ 0 };
    double bottom_offset{ 0 };
    double right_offset{ 0 };
    double part_offset{ 0 };
    double sheet_width{ 0 };
    double sheet_height{ 0 };
    size_t max_time{ 0 };
    std::vector<nesting::geo::Polygon_with_holes_2> polygons;
    std::vector<uint32_t> items_rotations;
    std::vector<uint32_t> items_quantity;
    int circle_segments{ 128 };
    bool fast_mode{ false };
    bool use_clipper{ false };
    std::function<void(const Solution&)> h =
        std::bind(&Worker::hook, this, std::placeholders::_1);
    nesting::Preprocess* p{ nullptr };
    void hook(const Solution& solution) {
        emit resultReady(Solution(solution), p->simplified_items,
            p->original_items);
    }
    void doWork() {
        qDebug() << "doWork START";

        // 1. 预处理阶段：如果失败，不再继续往下跑，直接返回，避免空指针崩溃
        try {
            p = new nesting::Preprocess(nesting::preprocess(
                need_simplify, top_offset, left_offset, bottom_offset, right_offset,
                part_offset, sheet_width, sheet_height, polygons, items_rotations,
                items_quantity, circle_segments));
        }
        catch (const std::runtime_error& e) {
            emit sendMessage(e.what());
            p = nullptr;
        }

        // 预处理失败或用户要求退出，直接结束
        if (p == nullptr || isRequestQuit) {
            qDebug() << "doWork ABORT (preprocess failed or quit requested)";
            return;
        }

        // 2. 核心排样阶段
        try {
            // 使用 CircleNesting / 调度 / 阵列模式进行全局优化（内部会调用局部精修）
            nesting::start_ga(max_time, p->layout, h, &isRequestQuit, fast_mode, use_clipper);
        }
        catch (const std::runtime_error& e) {
            emit sendMessage(e.what());
        }

        delete p;
        p = nullptr;
        qDebug() << "doWork DONE";
    }
};