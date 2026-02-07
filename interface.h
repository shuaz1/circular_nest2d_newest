#pragma once

#include "nesting.h"
#include "postprocess.h"
#include "preprocess.h"

namespace nesting {

    struct Preprocess {
        Layout layout;
        std::vector<nesting::Item> original_items;
        std::vector<nesting::Item> simplified_items;
        Preprocess(Layout& _layout,
            std::vector<nesting::Item>& _original_items,
            std::vector<nesting::Item>& _simplified_items)
            : layout(_layout) {
            original_items.swap(_original_items);
            simplified_items.swap(_simplified_items);
        }
    };

    Preprocess preprocess(
        const bool need_simplify,
        const double top_offset,
        const double left_offset,
        const double bottom_offset,
        const double right_offset,
        const double part_offset,
        const double sheet_width,
        const double sheet_height,
        const std::vector<nesting::geo::Polygon_with_holes_2>& polygons,
        const std::vector<uint32_t>& items_rotations,
        const std::vector<uint32_t>& items_quantity,
        const size_t circle_segments);

    // 圆形板材高利用率排样入口
    void start(const size_t max_time,
        Layout& layout,
        std::function<void(const Solution&)> ProgressHandler,
        volatile bool* requestQuit);

    // 圆形板材高利用率排样算法
    // fast_mode = true 时，启用极速模式：大幅减少迭代与候选点数量，以速度优先
    // use_clipper = true 时，使用 Clipper 库进行几何运算（更快，2-10倍速度提升）
    void start_ga(const size_t max_time,
        Layout& layout,
        std::function<void(const Solution&)> ProgressHandler,
        volatile bool* requestQuit,
        bool fast_mode,
        bool use_clipper = false);
}  // namespace nesting
