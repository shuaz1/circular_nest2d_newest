#include "circle_nesting.h"
#include <chrono>
#include "nesting.h"
#include "algorithm.h"
#include "clipper_wrapper.h"
#include <CGAL/boolean_set_operations_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/number_utils.h>
#include <CGAL/Uncertain.h>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <random>
#include <set>
#include <unordered_map>
#include <QtDebug>

namespace nesting {

    // Robust conversion from CGAL::FT to double using intervals
    double safe_to_double(const geo::FT& v) {
        auto I = CGAL::to_interval(v);
        double a = I.first;
        double b = I.second;
        if (!std::isfinite(a) || !std::isfinite(b)) {
            return 0.0;
        }
        return 0.5 * (a + b);
    }

    // ============================================================================
    // 一组仅使用 double 的几何辅助函数，完全绕开 CGAL 的布尔运算，
    // 用于 CGAL 简化路径下的“近似重叠检测”，避免频繁触发
    // CGAL::Uncertain_conversion_exception。
    // ============================================================================

    // 射线法判断点是否在多边形内（边界算作在内）
    static bool point_in_polygon_double(
        double x, double y,
        const std::vector<std::pair<double, double>>& poly) {
        bool inside = false;
        size_t n = poly.size();
        if (n < 3) return false;
        for (size_t i = 0, j = n - 1; i < n; j = i++) {
            const auto& pi = poly[i];
            const auto& pj = poly[j];
            double xi = pi.first;
            double yi = pi.second;
            double xj = pj.first;
            double yj = pj.second;

            // 边界上的点：用一个小阈值检查
            double dx = xj - xi;
            double dy = yj - yi;
            double cross = (x - xi) * dy - (y - yi) * dx;
            double dot = (x - xi) * (x - xj) + (y - yi) * (y - yj);
            if (std::fabs(cross) < 1e-9 && dot <= 0.0) {
                return true;
            }

            bool intersect = ((yi > y) != (yj > y)) &&
                (x < (xj - xi) * (y - yi) / (yj - yi + 1e-18) + xi);
            if (intersect) {
                inside = !inside;
            }
        }
        return inside;
    }

    // 线段相交检测（含端点和共线重叠情况）
    static int orient(double ax, double ay, double bx, double by, double cx, double cy) {
        double v = (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
        if (v > 1e-9) return 1;
        if (v < -1e-9) return -1;
        return 0;
    }

    static bool on_segment(double ax, double ay, double bx, double by, double px, double py) {
        return std::min(ax, bx) - 1e-9 <= px && px <= std::max(ax, bx) + 1e-9 &&
            std::min(ay, by) - 1e-9 <= py && py <= std::max(ay, by) + 1e-9 &&
            orient(ax, ay, bx, by, px, py) == 0;
    }

    static bool segments_intersect_double(
        double ax, double ay, double bx, double by,
        double cx, double cy, double dx, double dy) {
        int o1 = orient(ax, ay, bx, by, cx, cy);
        int o2 = orient(ax, ay, bx, by, dx, dy);
        int o3 = orient(cx, cy, dx, dy, ax, ay);
        int o4 = orient(cx, cy, dx, dy, bx, by);

        if (o1 != o2 && o3 != o4) return true;
        if (o1 == 0 && on_segment(ax, ay, bx, by, cx, cy)) return true;
        if (o2 == 0 && on_segment(ax, ay, bx, by, dx, dy)) return true;
        if (o3 == 0 && on_segment(cx, cy, dx, dy, ax, ay)) return true;
        if (o4 == 0 && on_segment(cx, cy, dx, dy, bx, by)) return true;
        return false;
    }


    CircleNesting::CircleNesting(Layout& layout)
        : layout_(layout), best_utilization_(0.0), use_time_limit_(false) {
        if (layout_.sheets.empty() || layout_.sheets[0].type != Sheet::ShapeType::Circle) {
            throw std::runtime_error("CircleNesting requires a circular sheet");
        }
        current_diameter_ = safe_to_double(layout_.sheets[0].diameter);
    }

    void CircleNesting::set_time_limit_seconds(size_t seconds) {
        if (seconds == 0) {
            use_time_limit_ = false;
            return;
        }
        use_time_limit_ = true;
        deadline_ = std::chrono::steady_clock::now() + std::chrono::seconds(seconds);
    }

    void CircleNesting::set_hard_time_limit_seconds(size_t seconds) {
        if (seconds == 0) {
            use_hard_time_limit_ = false;
            return;
        }
        use_hard_time_limit_ = true;
        hard_deadline_ = std::chrono::steady_clock::now() + std::chrono::seconds(seconds);
    }

    void CircleNesting::set_quality_gate(double gate_utilization) {
        if (gate_utilization <= 0.0) {
            use_quality_gate_ = false;
            quality_gate_ = 0.0;
            return;
        }
        use_quality_gate_ = true;
        quality_gate_ = gate_utilization;
    }

    bool CircleNesting::should_stop(volatile bool* requestQuit) const {
        if (requestQuit && *requestQuit) {
            return true;
        }

        auto now = std::chrono::steady_clock::now();

        // 硬上限：无论是否达标，一旦超时必须停
        if (use_hard_time_limit_ && now >= hard_deadline_) {
            if (requestQuit) {
                *requestQuit = true;
            }
            return true;
        }

        // 软上限：在未达到质量门槛前忽略；达到门槛后才允许因软上限停止
        if (use_time_limit_ && now >= deadline_) {
            bool gate_reached = true;
            if (use_quality_gate_) {
                gate_reached = best_utilization_ >= quality_gate_;
            }
            if (gate_reached) {
                if (requestQuit) {
                    *requestQuit = true;
                }
                return true;
            }
        }
        return false;
    }

    bool CircleNesting::cgal_validate_no_overlap_for_parts(const std::vector<TransformedShape>& parts,
        volatile bool* requestQuit) const {
        if (parts.empty()) {
            return true;
        }

        const double radius = safe_to_double(layout_.sheets[0].diameter) / 2.0;
        const double center_x = radius;
        const double center_y = radius;
        const double radius_sq = radius * radius;

        std::vector<size_t> active;
        active.reserve(parts.size());
        for (size_t i = 0; i < parts.size(); ++i) {
            if (should_stop(requestQuit)) {
                return false;
            }
            try {
                auto bbox = parts[i].transformed.bbox();
                double cx = safe_to_double(bbox.xmin() + bbox.xmax()) / 2.0;
                double cy = safe_to_double(bbox.ymin() + bbox.ymax()) / 2.0;
                double dist_sq = (cx - center_x) * (cx - center_x) + (cy - center_y) * (cy - center_y);
                if (dist_sq > radius_sq * 1.5) {
                    continue;
                }
                active.push_back(i);
            }
            catch (...) {
                continue;
            }
        }

        const double overlap_tolerance = 1e-12;
        for (size_t ai = 0; ai < active.size(); ++ai) {
            if (should_stop(requestQuit)) {
                return false;
            }
            size_t i = active[ai];
            auto bbox_i = parts[i].transformed.bbox();

            for (size_t aj = ai + 1; aj < active.size(); ++aj) {
                if (should_stop(requestQuit)) {
                    return false;
                }
                size_t j = active[aj];
                auto bbox_j = parts[j].transformed.bbox();
                if (bbox_i.xmax() < bbox_j.xmin() || bbox_i.xmin() > bbox_j.xmax() ||
                    bbox_i.ymax() < bbox_j.ymin() || bbox_i.ymin() > bbox_j.ymax()) {
                    continue;
                }

                try {
                    geo::Polygon_set_2 ps1(geo::traits);
                    ps1.insert(parts[i].transformed.outer_boundary());

                    geo::Polygon_set_2 ps2(geo::traits);
                    ps2.insert(parts[j].transformed.outer_boundary());

                    ps1.intersection(ps2);
                    if (ps1.is_empty()) {
                        continue;
                    }

                    std::vector<geo::Polygon_with_holes_2> intersection;
                    intersection.reserve(4);
                    ps1.polygons_with_holes(std::back_inserter(intersection));

                    double overlap_area = 0.0;
                    for (const auto& p : intersection) {
                        overlap_area += safe_to_double(geo::pwh_area(p));
                        if (overlap_area > overlap_tolerance) {
                            return false;
                        }
                    }
                }
                catch (...) {
                    return false;
                }
            }
        }

        return true;
    }

    void CircleNesting::rollback_to_last_validated_best() {
        if (!have_validated_best_ || last_validated_best_result_.empty()) {
            return;
        }
        layout_.sheet_parts = last_validated_best_result_;
        layout_.best_result = last_validated_best_result_;
        layout_.best_utilization = last_validated_best_utilization_;
        best_utilization_ = last_validated_best_utilization_;
    }

    void CircleNesting::maybe_cgal_validate_and_checkpoint_best(volatile bool* requestQuit) {
        if (params_.geometry_library != Parameters::GeometryLibrary::Clipper) {
            return;
        }

        auto now = std::chrono::steady_clock::now();
        if (next_cgal_validation_time_.time_since_epoch().count() == 0) {
            next_cgal_validation_time_ = now;
        }

        if (now < next_cgal_validation_time_) {
            return;
        }

        next_cgal_validation_time_ = now + std::chrono::seconds(10);

        if (layout_.best_result.empty() || layout_.best_result[0].empty()) {
            return;
        }

        bool ok = cgal_validate_no_overlap_for_parts(layout_.best_result[0], requestQuit);
        if (ok) {
            have_validated_best_ = true;
            last_validated_best_utilization_ = layout_.best_utilization;
            last_validated_best_result_ = layout_.best_result;
            return;
        }

        rollback_to_last_validated_best();
    }

    void CircleNesting::set_parameters(const Parameters& params) {
        params_ = params;
    }

    double CircleNesting::calculate_theoretical_min_diameter() const {
        // 理论最小直径：sqrt(4 * 总面积 / PI)
        // 考虑利用率目标，实际最小直径 = sqrt(4 * 总面积 / (PI * target_utilization))
        double total_area = safe_to_double(layout_.area);
        double min_diameter = std::sqrt(4.0 * total_area / (Sheet::kPi * 0.85)); // 目标利用率按 85%
        return min_diameter;
    }

    void CircleNesting::update_circle_diameter(double diameter) {
        current_diameter_ = diameter;
        layout_.sheets[0] = Sheet::make_circle(geo::FT(diameter), layout_.sheets[0].segments);
        layout_.sheets[0].diameter = geo::FT(diameter);
    }

    double CircleNesting::calculate_utilization() const {
        if (layout_.sheet_parts.empty() || layout_.sheet_parts[0].empty()) {
            return 0.0;
        }

        try {
            // 计算圆形板材面积
            double radius = safe_to_double(layout_.sheets[0].diameter) / 2.0;
            double circle_area = Sheet::kPi * radius * radius;

            if (circle_area <= 0) {
                return 0.0;
            }

            // 获取圆形板材多边形
            geo::Polygon_with_holes_2 sheet_pwh = layout_.sheets[0].sheet;

            // 根据参数选择使用 Clipper 或 CGAL
            if (params_.geometry_library == Parameters::GeometryLibrary::Clipper) {
                // 使用 Clipper（快速）
                double union_area = 0.0;

                // 逐个计算每个零件在圆内的部分，然后合并
                std::vector<geo::Polygon_with_holes_2> union_polygons;

                // 只处理实际放置在圆内的零件（通过检查位置判断）
                double radius = safe_to_double(layout_.sheets[0].diameter) / 2.0;
                double center_x = radius;
                double center_y = radius;

                for (const auto& shape : layout_.sheet_parts[0]) {
                    try {
                        // 快速检查：零件的 bbox 中心是否在圆内
                        auto bbox = shape.transformed.bbox();
                        double cx = safe_to_double(bbox.xmin() + bbox.xmax()) / 2.0;
                        double cy = safe_to_double(bbox.ymin() + bbox.ymax()) / 2.0;
                        double dist_sq = (cx - center_x) * (cx - center_x) + (cy - center_y) * (cy - center_y);

                        // 如果零件中心明显在圆外，跳过（节省计算）
                        if (dist_sq > radius * radius * 1.5) {
                            continue;
                        }

                        // 计算零件与板材的交集（只计算在圆内的部分）
                        auto inters_result = geo::GeometryOperations::intersection(shape.transformed, sheet_pwh);

                        if (inters_result.is_empty || inters_result.polygons.empty()) {
                            continue; // 零件不在圆内，跳过
                        }

                        // 将交集添加到并集列表
                        for (const auto& inter_poly : inters_result.polygons) {
                            union_polygons.push_back(inter_poly);
                        }
                    }
                    catch (...) {
                        // 忽略单个零件的错误，继续处理下一个
                    }
                }

                // 合并所有多边形（计算并集）
                if (!union_polygons.empty()) {
                    // 如果只有一个多边形，直接计算面积
                    if (union_polygons.size() == 1) {
                        union_area = geo::GeometryOperations::area(union_polygons[0]);
                    }
                    else {
                        // 多个多边形，尝试合并
                        geo::Polygon_with_holes_2 current_union = union_polygons[0];
                        bool merge_success = true;

                        for (size_t i = 1; i < union_polygons.size(); ++i) {
                            try {
                                auto join_result = geo::GeometryOperations::join(current_union, union_polygons[i]);
                                if (!join_result.is_empty && !join_result.polygons.empty()) {
                                    // 取第一个（通常只有一个）
                                    current_union = join_result.polygons[0];
                                }
                                else {
                                    // 合并失败，累加面积（近似处理）
                                    merge_success = false;
                                    break;
                                }
                            }
                            catch (...) {
                                // 合并失败，累加面积（近似处理）
                                merge_success = false;
                                break;
                            }
                        }

                        if (merge_success) {
                            // 计算最终并集面积
                            try {
                                union_area = geo::GeometryOperations::area(current_union);
                            }
                            catch (...) {
                                // 如果计算失败，使用累加方式（近似）
                                union_area = 0.0;
                                for (const auto& poly : union_polygons) {
                                    try {
                                        union_area += geo::GeometryOperations::area(poly);
                                    }
                                    catch (...) {
                                        // 忽略错误
                                    }
                                }
                            }
                        }
                        else {
                            // 合并失败，使用累加方式（近似，可能高估）
                            union_area = 0.0;
                            for (const auto& poly : union_polygons) {
                                try {
                                    union_area += geo::GeometryOperations::area(poly);
                                }
                                catch (...) {
                                    // 忽略错误
                                }
                            }
                        }
                    }
                }

                // 利用率 = 实际覆盖面积（并集，去重叠）/ 圆形板材面积
                double utilization = union_area / circle_area;
                if (utilization < 0) utilization = 0;
                if (utilization > 1) utilization = 1;

                return utilization;
            }
            else {
                // 使用 CGAL（精确，默认）
                // 使用 Polygon_set_2 计算所有已放置零件的并集面积
                // 注意：Polygon_set_2 的 join 操作会自动处理重叠（去重叠），确保重叠部分只计算一次
                // 这符合约束2：刀头之间不能重叠（如果放置时遵守了约束，这里不应该有重叠，但计算时仍然去重叠）
                geo::Polygon_set_2 ps(geo::traits);

                for (const auto& shape : layout_.sheet_parts[0]) {
                    try {
                        // 约束1：只计算在圆内的部分（刀头不能超出板材）
                        // 计算零件与板材的交集（只计算在圆内的部分）
                        std::vector<geo::Polygon_with_holes_2> inters;
                        try {
                            CGAL::intersection(shape.transformed, sheet_pwh, std::back_inserter(inters));
                        }
                        catch (...) {
                            inters.clear();
                        }

                        // 如果交集为空，说明零件不在圆内，跳过（遵守约束1）
                        if (inters.empty()) {
                            continue;
                        }

                        // 约束2：将交集加入多边形集合（自动处理重叠，确保重叠部分只计算一次）
                        for (const auto& p : inters) {
                            try {
                                ps.join(p.outer_boundary());
                            }
                            catch (...) {
                                // 忽略错误，继续处理下一个
                            }
                        }
                    }
                    catch (...) {
                        // 忽略单个零件的错误，继续处理下一个
                    }
                }

                // 计算并集总面积
                std::vector<geo::Polygon_with_holes_2> unions;
                double union_area = 0.0;
                try {
                    ps.polygons_with_holes(std::back_inserter(unions));
                }
                catch (...) {
                    unions.clear();
                }

                for (const auto& p : unions) {
                    try {
                        union_area += safe_to_double(geo::pwh_area(p));
                    }
                    catch (...) {
                        // 忽略错误
                    }
                }

                // 利用率 = 实际覆盖面积（并集，去重叠）/ 圆形板材面积
                double utilization = union_area / circle_area;
                if (utilization < 0) utilization = 0;
                if (utilization > 1) utilization = 1;

                return utilization;
            }
        }
        catch (...) {
            return 0.0;
        }
    }

    // ============================================================================
    // 核心约束检查函数：确保零件不超出圆形板材且不重叠
    // 这是代码的底线，所有放置操作都必须通过此检查
    // 
    // 约束1：刀头不能超出板材（零件必须完全在圆形板材内）
    // 约束2：刀头之间不能重叠（零件之间不能有任何重叠）
    // ============================================================================
    bool CircleNesting::is_valid_placement(size_t shape_idx, const std::vector<size_t>& placed_indices) const {
        if (shape_idx >= layout_.sheet_parts[0].size()) {
            return false;
        }

        // 对于 CGAL 模式，优先走一套相对简单、稳定的检查逻辑，
        // 避免复杂容差和面积阈值导致“本来能放却被误判”的情况。
        if (params_.geometry_library == Parameters::GeometryLibrary::CGAL) {
            return is_valid_placement_cgal_simple(shape_idx, placed_indices);
        }

        const auto& shape = layout_.sheet_parts[0][shape_idx];

        // ========== 约束1：严格检查是否完全在圆内（不能超出圆形板材） ==========
        try {
            double radius = safe_to_double(layout_.sheets[0].diameter) / 2.0;
            double center_x = radius;
            double center_y = radius;
            double radius_sq = radius * radius;
            const double tolerance = 1e-6; // 允许的数值误差

            // 获取圆形板材多边形
            geo::Polygon_with_holes_2 sheet_pwh = layout_.sheets[0].sheet;

            // 方法1：检查所有顶点是否在圆内（快速检查）
            // 大幅放宽容差，避免数值误差导致误判
            // 使用相对容差：允许顶点距离半径的1%误差
            double vertex_tolerance = radius_sq * 0.01; // 允许1%的相对误差，而不是固定的小数值
            size_t vertex_count = 0;
            for (auto v = shape.transformed.outer_boundary().vertices_begin();
                v != shape.transformed.outer_boundary().vertices_end(); ++v) {
                vertex_count++;
                double dx = safe_to_double(v->x()) - center_x;
                double dy = safe_to_double(v->y()) - center_y;
                double dist_sq = dx * dx + dy * dy;
                // 使用相对容差：如果距离平方超过半径平方的1.01倍，才认为超出
                if (dist_sq > radius_sq * 1.01) {
                    qDebug() << "[DEBUG] is_valid_placement: 顶点超出圆形, shape_idx=" << shape_idx
                        << ", vertex=" << vertex_count << ", dist_sq=" << dist_sq
                        << ", radius_sq=" << radius_sq << ", threshold=" << (radius_sq * 1.01);
                    return false; // 顶点超出圆形
                }
            }
            // 顶点检查通过（不要在热点循环中打印日志，会严重拖慢）

            // 方法2：检查所有边上的点（采样检查，确保边不超出）
            // 使用相对容差
            size_t edge_count = 0;
            for (auto e = shape.transformed.outer_boundary().edges_begin();
                e != shape.transformed.outer_boundary().edges_end(); ++e) {
                edge_count++;
                // 在边的中点采样检查
                auto source = e->source();
                auto target = e->target();
                double mid_x = safe_to_double((source.x() + target.x()) / 2.0);
                double mid_y = safe_to_double((source.y() + target.y()) / 2.0);
                double dx = mid_x - center_x;
                double dy = mid_y - center_y;
                double dist_sq = dx * dx + dy * dy;
                // 使用相对容差：如果距离平方超过半径平方的1.01倍，才认为超出
                if (dist_sq > radius_sq * 1.01) {
                    qDebug() << "[DEBUG] is_valid_placement: 边超出圆形, shape_idx=" << shape_idx
                        << ", edge=" << edge_count << ", dist_sq=" << dist_sq;
                    return false; // 边超出圆形
                }
            }
            // 边检查通过（不要在热点循环中打印日志，会严重拖慢）

            // 方法3：使用布尔运算，检查多边形是否完全在圆内
            // 【临时禁用】暂时跳过布尔运算检查，仅使用顶点和边的快速检查
            // 这样可以避免布尔运算的数值误差导致误判
            /*
            try {
                double intersection_area = 0.0;
                double shape_area = safe_to_double(geo::pwh_area(shape.transformed));

                // 如果面积太小，跳过精确检查（避免数值误差）
                if (shape_area < tolerance) {
                    // 面积太小，使用快速检查结果
                } else {
                    if (params_.geometry_library == Parameters::GeometryLibrary::Clipper) {
                        // 使用 Clipper
                        auto inters_result = geo::GeometryOperations::intersection(shape.transformed, sheet_pwh);
                        intersection_area = inters_result.total_area;
                    } else {
                        // 使用 CGAL
                        std::vector<geo::Polygon_with_holes_2> intersection;
                        CGAL::intersection(shape.transformed, sheet_pwh, std::back_inserter(intersection));

                        for (const auto& poly : intersection) {
                            try {
                                intersection_area += safe_to_double(geo::pwh_area(poly));
                            }
                            catch (...) {
                                // 忽略错误
                            }
                        }
                    }

                    // 如果交集面积小于多边形面积（超过容差），说明有部分超出圆形
                    // 使用相对容差，避免小面积时的数值误差
                    // 放宽容差，避免数值误差导致误判（特别是对于复杂形状）
                    double min_area_ratio = 0.90; // 统一放宽到90%，避免过于严格
                    double area_ratio = (shape_area > tolerance) ? (intersection_area / shape_area) : 1.0;
                    if (area_ratio < min_area_ratio) {
                        return false;
                    }
                }
            }
            catch (...) {
                // 如果布尔运算失败，使用保守策略：只检查顶点和边（已在上面完成）
                // 如果顶点和边都在圆内，通常整个多边形也在圆内
                // 不返回 false，继续使用快速检查的结果
            }
            */

            // 检查孔（如果有）
            // 使用相对容差
            size_t hole_count = 0;
            for (auto h = shape.transformed.holes_begin(); h != shape.transformed.holes_end(); ++h) {
                hole_count++;
                for (auto v = h->vertices_begin(); v != h->vertices_end(); ++v) {
                    double dx = safe_to_double(v->x()) - center_x;
                    double dy = safe_to_double(v->y()) - center_y;
                    double dist_sq = dx * dx + dy * dy;
                    // 使用相对容差：如果距离平方超过半径平方的1.01倍，才认为超出
                    if (dist_sq > radius_sq * 1.01) {
                        qDebug() << "[DEBUG] is_valid_placement: 孔超出圆形, shape_idx=" << shape_idx
                            << ", hole=" << hole_count;
                        return false;
                    }
                }
            }
            // 孔检查通过（不要在热点循环中打印日志，会严重拖慢）
        }
        catch (...) {
            return false;
        }

        // ========== 约束2：检查是否与已放置零件重叠（不能重叠） ==========
        // 优化：先用 bbox 快速排除，再用精确检测
        auto bbox_i = shape.transformed.bbox();

        for (size_t j : placed_indices) {
            if (j == shape_idx) continue;

            try {
                const auto& shape_j = layout_.sheet_parts[0][j];

                // 快速检测：bbox 不相交则必定不重叠
                auto bbox_j = shape_j.transformed.bbox();
                if (bbox_i.xmax() < bbox_j.xmin() || bbox_i.xmin() > bbox_j.xmax() ||
                    bbox_i.ymax() < bbox_j.ymin() || bbox_i.ymin() > bbox_j.ymax()) {
                    continue; // bbox 不相交，跳过精确检测
                }

                // 精确检测：布尔运算（根据参数选择 Clipper 或 CGAL）
                double overlap_area = 0.0;
                bool check_succeeded = false;

                if (params_.geometry_library == Parameters::GeometryLibrary::Clipper) {
                    // 使用 Clipper，但总是准备回退到 CGAL 作为验证
                    bool clipper_ok = false;
                    try {
                        auto inters_result = geo::GeometryOperations::intersection(
                            shape.transformed, shape_j.transformed);

                        // 检查结果：如果 polygons 为空或 is_empty 为 true，说明没有重叠
                        if (inters_result.polygons.empty() || inters_result.is_empty) {
                            // 没有重叠
                            check_succeeded = true;
                            overlap_area = 0.0;
                            clipper_ok = true;
                        }
                        else {
                            // 有重叠，计算重叠面积
                            overlap_area = inters_result.total_area;
                            check_succeeded = true;
                            clipper_ok = true;
                        }
                    }
                    catch (...) {
                        // Clipper 执行失败
                        clipper_ok = false;
                    }

                    // 如果 Clipper 失败或结果可疑，回退到 CGAL 进行验证
                    if (!clipper_ok || (!check_succeeded)) {
                        try {
                            geo::Polygon_set_2 ps1(geo::traits);
                            ps1.insert(shape.transformed.outer_boundary());

                            geo::Polygon_set_2 ps2(geo::traits);
                            ps2.insert(shape_j.transformed.outer_boundary());

                            ps1.intersection(ps2);

                            if (!ps1.is_empty()) {
                                std::vector<geo::Polygon_with_holes_2> intersection;
                                intersection.reserve(4);
                                ps1.polygons_with_holes(std::back_inserter(intersection));

                                overlap_area = 0.0;
                                for (const auto& poly : intersection) {
                                    overlap_area += safe_to_double(geo::pwh_area(poly));
                                }
                            }
                            else {
                                overlap_area = 0.0;
                            }
                            check_succeeded = true;
                        }
                        catch (...) {
                            // CGAL 也失败，使用保守策略：如果 bbox 相交，假设可能有重叠
                            // 但这里 bbox 已经相交（否则不会到这里），所以返回 false
                            return false;
                        }
                    }
                }
                else {
                    // 使用 CGAL
                    try {
                        geo::Polygon_set_2 ps1(geo::traits);
                        ps1.insert(shape.transformed.outer_boundary());

                        geo::Polygon_set_2 ps2(geo::traits);
                        ps2.insert(shape_j.transformed.outer_boundary());

                        ps1.intersection(ps2);

                        if (!ps1.is_empty()) {
                            std::vector<geo::Polygon_with_holes_2> intersection;
                            intersection.reserve(4);
                            ps1.polygons_with_holes(std::back_inserter(intersection));

                            for (const auto& poly : intersection) {
                                try {
                                    overlap_area += safe_to_double(geo::pwh_area(poly));
                                }
                                catch (const CGAL::Uncertain_conversion_exception&) {
                                    // CGAL 转换异常，忽略这个多边形，继续处理其他
                                }
                                catch (...) {
                                    // 其他异常也忽略
                                }
                            }
                        }
                        check_succeeded = true;
                    }
                    catch (const CGAL::Uncertain_conversion_exception&) {
                        // CGAL 转换异常，使用保守策略
                        return false;
                    }
                    catch (...) {
                        // CGAL 失败，保守策略：如果 bbox 相交，假设可能有重叠
                        return false;
                    }
                }

                // 如果检测成功且发现重叠，返回 false（严格不重叠）
                const double overlap_tolerance = 1e-12;
                if (check_succeeded && overlap_area > overlap_tolerance) {
                    return false;
                }
            }
            catch (...) {
                return false;
            }
        }
        return true;
    }

    // ============================================================================
    // 几何尺寸预检查：判断零件在当前圆直径下是否“无论如何都放不下”
    // 简单策略：使用零件外轮廓的 bbox 对角线作为近似直径，与圆板直径比较。
    // 如果 bbox 对角线明显大于圆直径（留出一点安全裕度），就认为永远放不下。
    // 这样可以避免在明显不可能的零件上浪费大量候选搜索时间。
    // ============================================================================
    bool CircleNesting::can_never_fit_in_circle(size_t shape_idx) const {
        if (shape_idx >= layout_.sheet_parts[0].size()) {
            return true;
        }

        try {
            const auto& shape = layout_.sheet_parts[0][shape_idx];
            const auto& poly = *shape.base;
            auto bbox = poly.bbox();

            double width = safe_to_double(bbox.xmax() - bbox.xmin());
            double height = safe_to_double(bbox.ymax() - bbox.ymin());
            if (width <= 0.0 || height <= 0.0) {
                return true;
            }

            double diag = std::sqrt(width * width + height * height); // 近似零件的最小包围圆直径
            double sheet_d = safe_to_double(layout_.sheets[0].diameter);

            // 留出 2% 的安全裕度：如果零件最小包围圆直径仍然大于圆板直径的 1.02 倍，则认为绝对放不下
            if (diag > sheet_d * 1.02) {
                qDebug() << "[DEBUG] can_never_fit_in_circle: 零件几何尺寸大于圆板, shape_idx="
                    << shape_idx << ", diag=" << diag << ", sheet_d=" << sheet_d;
                return true;
            }
        }
        catch (...) {
            // 出现异常时，宁可保守认为可能放得下，以免误杀
            return false;
        }

        return false;
    }

    // ============================================================================
    // CGAL 简化版约束检查：
    // - 约束1：顶点/孔顶点必须在圆内（使用较小的相对容差）
    // - 约束2：使用 Polygon_set_2 判断是否与已放置零件有交集（有交集则视为重叠）
    // 相比完整版本，这里不再计算重叠面积阈值，只要有明显交集就认为不合法，
    // 目的是先让 CGAL 模式“能稳定跑完”，作为对比基线。
    // ============================================================================
    bool CircleNesting::is_valid_placement_cgal_simple(
        size_t shape_idx,
        const std::vector<size_t>& placed_indices) const {
        if (shape_idx >= layout_.sheet_parts[0].size()) {
            return false;
        }

        const auto& shape = layout_.sheet_parts[0][shape_idx];

        // ===== 约束1：在圆内（使用相对较小的容差，避免把真正在外面的零件放进来）=====
        try {
            double radius = safe_to_double(layout_.sheets[0].diameter) / 2.0;
            double center_x = radius;
            double center_y = radius;
            double radius_sq = radius * radius;

            // 顶点检查
            for (auto v = shape.transformed.outer_boundary().vertices_begin();
                v != shape.transformed.outer_boundary().vertices_end(); ++v) {
                double dx = safe_to_double(v->x()) - center_x;
                double dy = safe_to_double(v->y()) - center_y;
                double dist_sq = dx * dx + dy * dy;
                // 只允许非常小的浮点误差（0.1%）
                if (dist_sq > radius_sq * 1.001) {
                    return false;
                }
            }

            // 孔顶点检查
            for (auto h = shape.transformed.holes_begin();
                h != shape.transformed.holes_end(); ++h) {
                for (auto v = h->vertices_begin(); v != h->vertices_end(); ++v) {
                    double dx = safe_to_double(v->x()) - center_x;
                    double dy = safe_to_double(v->y()) - center_y;
                    double dist_sq = dx * dx + dy * dy;
                    if (dist_sq > radius_sq * 1.001) {
                        return false;
                    }
                }
            }
        }
        catch (...) {
            // 圆内检查失败时，保守起见认为不合法
            return false;
        }

        // ===== 约束2：不能与已放置零件重叠（仅使用 double 近似检测，避免 CGAL 布尔运算）=====
        auto bbox_i = shape.transformed.bbox();

        // 先把当前零件外轮廓转换成 double 多边形
        std::vector<std::pair<double, double>> poly_i;
        for (auto v = shape.transformed.outer_boundary().vertices_begin();
            v != shape.transformed.outer_boundary().vertices_end(); ++v) {
            poly_i.emplace_back(safe_to_double(v->x()), safe_to_double(v->y()));
        }

        for (size_t j : placed_indices) {
            if (j == shape_idx) continue;

            const auto& shape_j = layout_.sheet_parts[0][j];

            // bbox 快速排除
            auto bbox_j = shape_j.transformed.bbox();
            if (bbox_i.xmax() < bbox_j.xmin() || bbox_i.xmin() > bbox_j.xmax() ||
                bbox_i.ymax() < bbox_j.ymin() || bbox_i.ymin() > bbox_j.ymax()) {
                continue;
            }

            // 将已放置零件外轮廓转换为 double 多边形
            std::vector<std::pair<double, double>> poly_j;
            for (auto v = shape_j.transformed.outer_boundary().vertices_begin();
                v != shape_j.transformed.outer_boundary().vertices_end(); ++v) {
                poly_j.emplace_back(safe_to_double(v->x()), safe_to_double(v->y()));
            }

            // 1) 顶点包含测试：A 顶点在 B 内，或 B 顶点在 A 内，都视为重叠
            for (const auto& p : poly_i) {
                if (point_in_polygon_double(p.first, p.second, poly_j)) {
                    return false;
                }
            }
            for (const auto& p : poly_j) {
                if (point_in_polygon_double(p.first, p.second, poly_i)) {
                    return false;
                }
            }

            // 2) 边与边相交测试
            size_t ni = poly_i.size();
            size_t nj = poly_j.size();
            if (ni >= 2 && nj >= 2) {
                for (size_t a = 0; a < ni; ++a) {
                    size_t a2 = (a + 1) % ni;
                    double ax = poly_i[a].first;
                    double ay = poly_i[a].second;
                    double bx = poly_i[a2].first;
                    double by = poly_i[a2].second;

                    for (size_t b = 0; b < nj; ++b) {
                        size_t b2 = (b + 1) % nj;
                        double cx = poly_j[b].first;
                        double cy = poly_j[b].second;
                        double dx = poly_j[b2].first;
                        double dy = poly_j[b2].second;

                        if (segments_intersect_double(ax, ay, bx, by, cx, cy, dx, dy)) {
                            return false;
                        }
                    }
                }
            }
        }

        return true;
    }

    std::vector<geo::Point_2> CircleNesting::generate_circular_candidates(
        size_t shape_idx,
        const std::vector<size_t>& placed_indices,
        const geo::Polygon_2& ifr) {

        std::vector<geo::Point_2> candidates;

        if (shape_idx >= layout_.sheet_parts[0].size()) {
            return candidates;
        }

        try {
            const auto& shape = layout_.sheet_parts[0][shape_idx];
            double radius = safe_to_double(layout_.sheets[0].diameter) / 2.0;
            double center_x = radius;
            double center_y = radius;

            // 方法1：基于NFP的完美点（如果可用）
            if (!placed_indices.empty()) {
                CandidatePoints c(placed_indices.size());
                c.set_boundary(ifr);

                auto rotation_i = shape.get_rotation();
                for (size_t j : placed_indices) {
                    const auto& shape_j = layout_.sheet_parts[0][j];
                    auto rotation_j = shape_j.get_rotation();
                    auto& nfp = comp_nfp(shape_j.base, rotation_j, shape_j.allowed_rotations,
                        shape.base, rotation_i, shape.allowed_rotations, layout_);
                    c.nfps.push_back(&nfp);
                    c.translations.push_back(shape_j.get_translate());
                    c.translate_x.push_back(shape_j.get_translate_double_x());
                }

                c.initialize();
                auto perfect_points = c.get_perfect_points();

                // 过滤：只保留在圆内的点
                std::vector<geo::Point_2> perfect_in_circle;
                perfect_in_circle.reserve(perfect_points.size());
                for (const auto& pt : perfect_points) {
                    double dx = safe_to_double(pt.x()) - center_x;
                    double dy = safe_to_double(pt.y()) - center_y;
                    double dist_sq = dx * dx + dy * dy;
                    if (dist_sq <= radius * radius) {
                        perfect_in_circle.push_back(pt);
                    }
                }

                const size_t max_keep = std::max<size_t>(1, params_.max_candidate_points / 2);
                if (perfect_in_circle.size() > max_keep) {
                    std::sort(perfect_in_circle.begin(), perfect_in_circle.end(), [&](const geo::Point_2& a, const geo::Point_2& b) {
                        double dax = safe_to_double(a.x()) - center_x;
                        double day = safe_to_double(a.y()) - center_y;
                        double dbx = safe_to_double(b.x()) - center_x;
                        double dby = safe_to_double(b.y()) - center_y;
                        return (dax * dax + day * day) < (dbx * dbx + dby * dby);
                        });
                    perfect_in_circle.resize(max_keep);
                }

                candidates.insert(candidates.end(), perfect_in_circle.begin(), perfect_in_circle.end());
            }

            // 方法2：径向候选点生成（如果启用）- 增强版
            const size_t reserve_for_ifr = std::min<size_t>(params_.max_candidate_points, std::max<size_t>(8, params_.max_candidate_points / 5));
            if (params_.use_radial_candidates && candidates.size() + reserve_for_ifr < params_.max_candidate_points) {
                auto bbox = shape.transformed.bbox();
                double shape_width = safe_to_double(bbox.xmax() - bbox.xmin());
                double shape_height = safe_to_double(bbox.ymax() - bbox.ymin());
                double shape_diagonal = std::sqrt(shape_width * shape_width + shape_height * shape_height);
                const bool allow_near_edge = (params_.use_binary_search || params_.diameter_optimization_iterations > 0);
                const double edge_ratio = allow_near_edge ? 0.999 : 0.98;

                // 生成径向候选点：从中心到边缘，多个角度（使用参数配置）
                size_t radial_layers = params_.radial_layers;
                size_t angles_per_layer = params_.angles_per_layer;

                // 优先生成靠近中心的候选点（更紧凑）
                const size_t edge_layers = allow_near_edge ? std::min<size_t>(6, std::max<size_t>(1, radial_layers / 4)) : 0;
                const size_t core_layers = (radial_layers > edge_layers) ? (radial_layers - edge_layers) : radial_layers;

                for (size_t layer = 0; layer < core_layers; ++layer) {
                    // 非线性分布：中心区域更密集
                    double layer_ratio = (core_layers > 1) ? (double(layer) / (core_layers - 1)) : 0.0;
                    double r_ratio = 0.15 + 0.75 * (layer_ratio * layer_ratio); // 平方分布，中心更密集
                    double r = radius * r_ratio;

                    for (size_t a = 0; a < angles_per_layer; ++a) {
                        double angle = 2.0 * Sheet::kPi * double(a) / angles_per_layer;
                        // 添加小角度偏移以增加候选点密度
                        for (int offset = -1; offset <= 1; offset += 2) {
                            double angle_offset = angle + offset * (2.0 * Sheet::kPi / angles_per_layer / 4.0);
                            double x = center_x + r * std::cos(angle_offset) - shape_width / 2.0;
                            double y = center_y + r * std::sin(angle_offset) - shape_height / 2.0;

                            // 确保在圆内
                            double max_dist = std::sqrt(
                                std::max((x + shape_width - center_x) * (x + shape_width - center_x),
                                    (x - center_x) * (x - center_x)) +
                                std::max((y + shape_height - center_y) * (y + shape_height - center_y),
                                    (y - center_y) * (y - center_y)));

                            if (max_dist <= radius * edge_ratio) { // 留出安全边距
                                candidates.push_back(geo::Point_2(geo::FT(x), geo::FT(y)));
                            }

                            if (candidates.size() + reserve_for_ifr >= params_.max_candidate_points) {
                                break;
                            }
                        }
                        if (candidates.size() + reserve_for_ifr >= params_.max_candidate_points) {
                            break;
                        }
                    }
                    if (candidates.size() + reserve_for_ifr >= params_.max_candidate_points) {
                        break;
                    }
                }

                if (allow_near_edge && candidates.size() + reserve_for_ifr < params_.max_candidate_points) {
                    for (size_t layer = 0; layer < edge_layers; ++layer) {
                        double t = (edge_layers > 1) ? (double(layer) / (edge_layers - 1)) : 1.0;
                        double r_ratio = 0.90 + 0.099 * t;
                        double r = radius * r_ratio;
                        for (size_t a = 0; a < angles_per_layer; ++a) {
                            double angle = 2.0 * Sheet::kPi * double(a) / angles_per_layer;
                            for (int offset = -1; offset <= 1; offset += 2) {
                                double angle_offset = angle + offset * (2.0 * Sheet::kPi / angles_per_layer / 4.0);
                                double x = center_x + r * std::cos(angle_offset) - shape_width / 2.0;
                                double y = center_y + r * std::sin(angle_offset) - shape_height / 2.0;
                                double max_dist = std::sqrt(
                                    std::max((x + shape_width - center_x) * (x + shape_width - center_x),
                                        (x - center_x) * (x - center_x)) +
                                    std::max((y + shape_height - center_y) * (y + shape_height - center_y),
                                        (y - center_y) * (y - center_y)));
                                if (max_dist <= radius * edge_ratio) {
                                    candidates.push_back(geo::Point_2(geo::FT(x), geo::FT(y)));
                                }
                                if (candidates.size() + reserve_for_ifr >= params_.max_candidate_points) {
                                    break;
                                }
                            }
                            if (candidates.size() + reserve_for_ifr >= params_.max_candidate_points) {
                                break;
                            }
                        }
                        if (candidates.size() + reserve_for_ifr >= params_.max_candidate_points) {
                            break;
                        }
                    }
                }
            }

            // 方法3：IFR边界上的点
            if (candidates.size() < params_.max_candidate_points && !ifr.is_empty()) {
                for (auto v = ifr.vertices_begin(); v != ifr.vertices_end(); ++v) {
                    double dx = safe_to_double(v->x()) - center_x;
                    double dy = safe_to_double(v->y()) - center_y;
                    double dist_sq = dx * dx + dy * dy;
                    if (dist_sq <= radius * radius) {
                        candidates.push_back(*v);
                    }
                    if (candidates.size() >= params_.max_candidate_points) {
                        break;
                    }
                }
            }

            if (candidates.size() < params_.max_candidate_points && !ifr.is_empty()) {
                auto add_if_ok = [&](const geo::Point_2& p) {
                    double dx = safe_to_double(p.x()) - center_x;
                    double dy = safe_to_double(p.y()) - center_y;
                    double dist_sq = dx * dx + dy * dy;
                    if (dist_sq <= radius * radius) {
                        candidates.push_back(p);
                    }
                    };

                auto interp = [&](const geo::Point_2& a, const geo::Point_2& b, double t) -> geo::Point_2 {
                    const geo::FT tt(t);
                    const geo::FT one(1.0);
                    const geo::FT omt = one - tt;
                    return geo::Point_2{ a.x() * omt + b.x() * tt, a.y() * omt + b.y() * tt };
                    };

                auto vb = ifr.vertices_begin();
                if (vb != ifr.vertices_end()) {
                    auto prev = vb;
                    auto cur = vb;
                    ++cur;
                    for (; cur != ifr.vertices_end(); ++cur) {
                        if (candidates.size() >= params_.max_candidate_points) {
                            break;
                        }
                        geo::Point_2 a = *prev;
                        geo::Point_2 b = *cur;
                        add_if_ok(interp(a, b, 0.5));
                        if (candidates.size() >= params_.max_candidate_points) {
                            break;
                        }
                        add_if_ok(interp(a, b, 0.25));
                        if (candidates.size() >= params_.max_candidate_points) {
                            break;
                        }
                        add_if_ok(interp(a, b, 0.75));
                        prev = cur;
                    }

                    if (candidates.size() < params_.max_candidate_points) {
                        geo::Point_2 a = *prev;
                        geo::Point_2 b = *vb;
                        add_if_ok(interp(a, b, 0.5));
                        if (candidates.size() < params_.max_candidate_points) {
                            add_if_ok(interp(a, b, 0.25));
                        }
                        if (candidates.size() < params_.max_candidate_points) {
                            add_if_ok(interp(a, b, 0.75));
                        }
                    }
                }
            }
        }
        catch (...) {
            // 如果生成失败，返回空列表
        }

        return candidates;
    }

    bool CircleNesting::place_shape_with_nfp(size_t shape_idx,
        const std::vector<size_t>& placed_indices,
        bool try_all_rotations,
        volatile bool* requestQuit,
        bool allow_waste_minimization) {
        if (shape_idx >= layout_.sheet_parts[0].size()) {
            return false;
        }

        // 如果启用废料最小化且已有放置的零件，优先尝试在废料区域放置
        if (allow_waste_minimization && params_.use_waste_minimization && !placed_indices.empty()) {
            bool placed = place_shape_in_waste(shape_idx, placed_indices, try_all_rotations);
            if (placed) {
                return true;
            }
            // 如果废料区域放置失败，继续使用普通NFP方法
        }

        auto& shape = layout_.sheet_parts[0][shape_idx];

        try {
            // 确保transformed是最新的
            shape.update();

            // 计算IFR（使用transformed，因为IFR需要知道当前旋转状态）
            qDebug() << "[DEBUG] place_shape_with_nfp: 开始, shape_idx=" << shape_idx
                << ", placed_indices.size()=" << placed_indices.size()
                << ", try_all_rotations=" << try_all_rotations;

            auto ifr = geo::comp_ifr(layout_.sheets[0].sheet, shape.transformed);
            qDebug() << "[DEBUG] place_shape_with_nfp: IFR计算完成, is_empty=" << ifr.is_empty();

            // 生成候选点（即使 IFR 为空，也尝试生成径向候选点）
            std::vector<geo::Point_2> candidates;
            if (!ifr.is_empty()) {
                candidates = generate_circular_candidates(shape_idx, placed_indices, ifr);
                qDebug() << "[DEBUG] place_shape_with_nfp: 从IFR生成候选点, count=" << candidates.size();
            }
            else {
                qDebug() << "[DEBUG] place_shape_with_nfp: IFR为空，将尝试生成径向候选点";
            }

            // 如果候选点为空或 IFR 为空，尝试生成径向候选点作为备选
            if (candidates.empty() && params_.use_radial_candidates) {
                // 直接生成径向候选点，不依赖 IFR
                double radius = safe_to_double(layout_.sheets[0].diameter) / 2.0;
                double center_x = radius;
                double center_y = radius;
                auto bbox = shape.transformed.bbox();
                double shape_width = safe_to_double(bbox.xmax() - bbox.xmin());
                double shape_height = safe_to_double(bbox.ymax() - bbox.ymin());
                const bool allow_near_edge = (params_.use_binary_search || params_.diameter_optimization_iterations > 0);
                const double edge_ratio = allow_near_edge ? 0.999 : 0.98;

                size_t radial_layers = params_.radial_layers;
                size_t angles_per_layer = params_.angles_per_layer;

                size_t generated_candidates = 0;
                const size_t max_candidates_per_shape = params_.max_candidate_points;

                const size_t edge_layers = allow_near_edge ? std::min<size_t>(6, std::max<size_t>(1, radial_layers / 4)) : 0;
                const size_t core_layers = (radial_layers > edge_layers) ? (radial_layers - edge_layers) : radial_layers;

                for (size_t layer = 0; layer < core_layers; ++layer) {
                    if (should_stop(requestQuit)) {
                        return false;
                    }
                    double layer_ratio = (core_layers > 1) ? (double(layer) / (core_layers - 1)) : 0.0;
                    double r_ratio = 0.15 + 0.75 * (layer_ratio * layer_ratio);
                    double r = radius * r_ratio;

                    for (size_t a = 0; a < angles_per_layer; ++a) {
                        double angle = 2.0 * Sheet::kPi * double(a) / angles_per_layer;
                        for (int offset = -1; offset <= 1; offset += 2) {
                            double angle_offset = angle + offset * (2.0 * Sheet::kPi / angles_per_layer / 4.0);
                            double x = center_x + r * std::cos(angle_offset) - shape_width / 2.0;
                            double y = center_y + r * std::sin(angle_offset) - shape_height / 2.0;

                            double max_dist = std::sqrt(
                                std::max((x + shape_width - center_x) * (x + shape_width - center_x),
                                    (x - center_x) * (x - center_x)) +
                                std::max((y + shape_height - center_y) * (y + shape_height - center_y),
                                    (y - center_y) * (y - center_y)));

                            if (max_dist <= radius * edge_ratio) {
                                candidates.push_back(geo::Point_2(geo::FT(x), geo::FT(y)));
                                ++generated_candidates;
                            }

                            if (generated_candidates >= max_candidates_per_shape) {
                                break;
                            }
                        }
                        if (generated_candidates >= max_candidates_per_shape) {
                            break;
                        }
                    }
                    if (generated_candidates >= max_candidates_per_shape) {
                        break;
                    }
                }

                if (allow_near_edge && generated_candidates < max_candidates_per_shape) {
                    for (size_t layer = 0; layer < edge_layers; ++layer) {
                        if (should_stop(requestQuit)) {
                            return false;
                        }
                        double t = (edge_layers > 1) ? (double(layer) / (edge_layers - 1)) : 1.0;
                        double r_ratio = 0.90 + 0.099 * t;
                        double r = radius * r_ratio;
                        for (size_t a = 0; a < angles_per_layer; ++a) {
                            double angle = 2.0 * Sheet::kPi * double(a) / angles_per_layer;
                            for (int offset = -1; offset <= 1; offset += 2) {
                                double angle_offset = angle + offset * (2.0 * Sheet::kPi / angles_per_layer / 4.0);
                                double x = center_x + r * std::cos(angle_offset) - shape_width / 2.0;
                                double y = center_y + r * std::sin(angle_offset) - shape_height / 2.0;
                                double max_dist = std::sqrt(
                                    std::max((x + shape_width - center_x) * (x + shape_width - center_x),
                                        (x - center_x) * (x - center_x)) +
                                    std::max((y + shape_height - center_y) * (y + shape_height - center_y),
                                        (y - center_y) * (y - center_y)));
                                if (max_dist <= radius * edge_ratio) {
                                    candidates.push_back(geo::Point_2(geo::FT(x), geo::FT(y)));
                                    ++generated_candidates;
                                }
                                if (generated_candidates >= max_candidates_per_shape) {
                                    break;
                                }
                            }
                            if (generated_candidates >= max_candidates_per_shape) {
                                break;
                            }
                        }
                        if (generated_candidates >= max_candidates_per_shape) {
                            break;
                        }
                    }
                }
            }

            if (candidates.empty()) {
                qDebug() << "[DEBUG] place_shape_with_nfp: 候选点为空，返回false";
                return false;
            }

            qDebug() << "[DEBUG] place_shape_with_nfp: 开始尝试候选点, total=" << candidates.size();

            // 尝试每个候选点，选择最紧凑的位置（最靠近中心）
            double radius = safe_to_double(layout_.sheets[0].diameter) / 2.0;
            double center_x = radius;
            double center_y = radius;

            geo::Point_2 best_point;
            uint32_t best_rotation = shape.get_rotation();
            double best_score = std::numeric_limits<double>::max();
            bool found = false;

            uint32_t original_rotation = shape.get_rotation();
            const uint32_t allowed_for_bias = std::max<uint32_t>(1, shape.allowed_rotations);

            // 如果需要尝试所有旋转角度
            std::vector<uint32_t> rotations_to_try;
            if (try_all_rotations && params_.try_all_rotations) {
                // 尝试所有允许的旋转角度（如果allowed_rotations=4，则尝试0°, 90°, 180°, 270°）
                // 但不超过max_rotation_attempts限制（避免旋转次数过多）
                const uint32_t allowed = std::max<uint32_t>(1, shape.allowed_rotations);
                const size_t max_rot = std::min(static_cast<size_t>(allowed),
                    static_cast<size_t>(std::max<size_t>(1, params_.max_rotation_attempts)));
                auto push_unique = [&](uint32_t r) {
                    r = r % allowed;
                    if (std::find(rotations_to_try.begin(), rotations_to_try.end(), r) == rotations_to_try.end()) {
                        rotations_to_try.push_back(r);
                    }
                    };

                if (allowed >= 2) {
                    push_unique(original_rotation);
                    push_unique(original_rotation + allowed / 2);
                }
                else {
                    push_unique(original_rotation);
                }
                push_unique(0);
                if (allowed >= 4) {
                    push_unique(allowed / 4);
                    push_unique((allowed * 3) / 4);
                }

                for (uint32_t rot = 0; rotations_to_try.size() < max_rot && rot < allowed; ++rot) {
                    push_unique(rot);
                }

                if (rotations_to_try.size() > max_rot) {
                    rotations_to_try.resize(max_rot);
                }
            }
            else {
                rotations_to_try.push_back(original_rotation);
            }

            // 对每个零件限制总尝试次数，避免在单个零件上耗费过多时间
            const size_t max_total_attempts_per_shape =
                static_cast<size_t>(params_.max_candidate_points) *
                static_cast<size_t>(std::max<uint32_t>(1, shape.allowed_rotations));
            size_t total_attempts = 0;

            for (uint32_t rot : rotations_to_try) {
                if (should_stop(requestQuit)) {
                    return false;
                }
                shape.set_rotation(rot);
                shape.update();

                // 重新计算IFR（因为旋转改变了形状）
                auto ifr_rotated = geo::comp_ifr(layout_.sheets[0].sheet, shape.transformed);
                if (ifr_rotated.is_empty()) {
                    continue;
                }

                // 如果旋转了，重新生成候选点
                std::vector<geo::Point_2> candidates_for_rotation = candidates;
                if (rot != original_rotation) {
                    candidates_for_rotation = generate_circular_candidates(shape_idx, placed_indices, ifr_rotated);
                }

                double neighbor_threshold = 0.0;
                try {
                    auto bbox_r = shape.transformed.bbox();
                    double w = safe_to_double(bbox_r.xmax() - bbox_r.xmin());
                    double h = safe_to_double(bbox_r.ymax() - bbox_r.ymin());
                    double diag = std::sqrt(w * w + h * h);
                    neighbor_threshold = std::max(1e-9, diag * 3.5);
                }
                catch (...) {
                    neighbor_threshold = 0.0;
                }

                for (const auto& candidate : candidates_for_rotation) {
                    if (should_stop(requestQuit)) {
                        return false;
                    }
                    if (total_attempts >= max_total_attempts_per_shape) {
                        // 达到单个零件的尝试上限，不再继续
                        qDebug() << "[DEBUG] place_shape_with_nfp: 超过尝试上限, shape_idx="
                            << shape_idx << ", total_attempts=" << total_attempts;
                        break;
                    }
                    ++total_attempts;
                    // 临时放置
                    auto old_x = shape.get_translate_ft_x();
                    auto old_y = shape.get_translate_ft_y();

                    shape.set_translate(candidate.x(), candidate.y());
                    shape.update();

                    // 检查是否有效
                    if (is_valid_placement(shape_idx, placed_indices)) {
                        // 计算评分：距离中心 + 空隙利用率
                        double dx = safe_to_double(candidate.x()) - center_x;
                        double dy = safe_to_double(candidate.y()) - center_y;
                        double dist_sq = dx * dx + dy * dy;

                        // 优先选择靠近中心的位置
                        double score = dist_sq;

                        double min_dist_to_placed = std::numeric_limits<double>::max();
                        double nearest_same_dist = std::numeric_limits<double>::max();
                        uint32_t nearest_same_rotation = 0;
                        for (size_t j : placed_indices) {
                            auto bbox_j = layout_.sheet_parts[0][j].transformed.bbox();
                            double cx_j = safe_to_double(bbox_j.xmin() + bbox_j.xmax()) / 2.0;
                            double cy_j = safe_to_double(bbox_j.ymin() + bbox_j.ymax()) / 2.0;
                            double dist_to_j = std::sqrt((dx + center_x - cx_j) * (dx + center_x - cx_j) +
                                (dy + center_y - cy_j) * (dy + center_y - cy_j));
                            min_dist_to_placed = std::min(min_dist_to_placed, dist_to_j);

                            if (layout_.sheet_parts[0][j].item_idx == shape.item_idx) {
                                if (dist_to_j < nearest_same_dist) {
                                    nearest_same_dist = dist_to_j;
                                    nearest_same_rotation = layout_.sheet_parts[0][j].get_rotation();
                                }
                            }
                        }

                        if (params_.prioritize_compact_positions) {
                            score = dist_sq * 0.7 + min_dist_to_placed * 0.3;
                        }

                        if (allowed_for_bias >= 2 && neighbor_threshold > 0.0 &&
                            nearest_same_dist < neighbor_threshold) {
                            uint32_t desired = (nearest_same_rotation + allowed_for_bias / 2) % allowed_for_bias;
                            double w = (neighbor_threshold - nearest_same_dist) / neighbor_threshold;
                            if (w < 0.0) w = 0.0;
                            if (w > 1.0) w = 1.0;

                            const double scale = (radius * radius + 1.0);
                            const double penalty = scale * 0.12 * w;
                            const double reward = scale * 0.06 * w;

                            if ((rot % allowed_for_bias) != desired) {
                                score += penalty;
                            }
                            else {
                                score -= reward;
                            }
                        }

                        if (score < best_score) {
                            best_score = score;
                            best_point = candidate;
                            best_rotation = rot;
                            found = true;
                        }
                    }

                    // 恢复
                    shape.set_translate(old_x, old_y);
                    shape.update();
                }
            }

            if (found) {
                qDebug() << "[DEBUG] place_shape_with_nfp: 成功找到位置, shape_idx=" << shape_idx
                    << ", rotation=" << best_rotation;
                shape.set_rotation(best_rotation);
                shape.set_translate(best_point.x(), best_point.y());
                shape.update();
                return true;
            }
            else {
                qDebug() << "[DEBUG] place_shape_with_nfp: 未找到有效位置, shape_idx=" << shape_idx;
                // 恢复原始旋转
                shape.set_rotation(original_rotation);
                shape.update();
            }

            return false;
        }
        catch (...) {
            return false;
        }
    }

    bool CircleNesting::try_place_all_parts(double diameter, volatile bool* requestQuit,
        std::function<void(const Layout&)>* progress_callback) {
        update_circle_diameter(diameter);

        // 重置所有零件位置：将未放置的零件放在板材外面（负坐标区域）
        // 这样在可视化时不会影响显示
        double radius = diameter / 2.0;
        double outside_offset = -radius * 2.0; // 放在板材外足够远的位置
        for (auto& shape : layout_.sheet_parts[0]) {
            shape.set_translate(geo::FT(outside_offset), geo::FT(outside_offset));
            shape.update();
        }
        if (params_.use_scheduling && params_.scheduling_attempts > 1) {
            schedule_best_order(diameter, requestQuit, progress_callback);
            double util_now = 0.0;
            try {
                util_now = calculate_utilization();
            }
            catch (...) {
                util_now = 0.0;
            }
            if (util_now > best_utilization_) {
                best_utilization_ = util_now;
                layout_.best_utilization = best_utilization_;
                layout_.best_result = layout_.sheet_parts;
            }
            return util_now > 0.0;
        }

        // 智能排序：考虑面积、形状复杂度、长宽比
        std::vector<size_t> indices;
        if (params_.use_smart_ordering) {
            indices = smart_order_parts();
        }
        else {
            indices.resize(layout_.poly_num);
            std::iota(indices.begin(), indices.end(), 0);
            std::sort(indices.begin(), indices.end(), [&](size_t i, size_t j) {
                try {
                    double area_i = safe_to_double(geo::pwh_area(*layout_.sheet_parts[0][i].base));
                    double area_j = safe_to_double(geo::pwh_area(*layout_.sheet_parts[0][j].base));
                    return area_i > area_j;
                }
                catch (...) {
                    return false;
                }
                });
        }

        std::vector<size_t> placed_indices;
        size_t lns_total_trials = 0;
        const size_t lns_max_trials = 120;
        size_t boosted_total_trials = 0;
        const size_t boosted_max_trials = 40;

        // 放置第一个零件：稍微远离中心，给其他零件留出空间
        if (!indices.empty()) {
            size_t first_idx = indices[0];
            if (can_never_fit_in_circle(first_idx)) {
                qDebug() << "[DEBUG] try_place_all_parts: 第一个零件几何尺寸大于圆板, shape_idx=" << first_idx;
                // 第一个零件本身放不下，直接返回失败，交给上层决定是否调整直径
                return false;
            }
            double radius = diameter / 2.0;
            auto& shape = layout_.sheet_parts[0][first_idx];

            shape.update();
            auto bbox = shape.transformed.bbox();
            double shape_width = safe_to_double(bbox.xmax() - bbox.xmin());
            double shape_height = safe_to_double(bbox.ymax() - bbox.ymin());

            shape.set_translate(geo::FT(radius - shape_width / 2.0),
                geo::FT(radius - shape_height / 2.0));
            shape.update();

            std::vector<size_t> empty_placed;
            if (!is_valid_placement(first_idx, empty_placed)) {
                if (!place_shape_with_nfp(first_idx, empty_placed, params_.try_all_rotations, requestQuit)) {
                    return false;
                }
            }

            placed_indices.push_back(first_idx);
        }

        // 依次放置剩余零件
        for (size_t i = 1; i < indices.size(); ++i) {
            if (should_stop(requestQuit)) {
                return false;
            }

            size_t shape_idx = indices[i];
            if (can_never_fit_in_circle(shape_idx)) {
                qDebug() << "[DEBUG] try_place_all_parts: 零件几何尺寸大于圆板, 跳过 shape_idx=" << shape_idx;
                continue;
            }
            bool placed = false;

            if (params_.try_all_rotations) {
                placed = place_shape_with_nfp(shape_idx, placed_indices, /*try_all_rotations=*/true, requestQuit);
            }
            else {
                // 完全不旋转的模式，保持原有行为
                placed = place_shape_with_nfp(shape_idx, placed_indices, /*try_all_rotations=*/false, requestQuit);
            }

            if (placed) {
                // 成功放置，添加到已放置列表
                placed_indices.push_back(shape_idx);
            }
            else {
                if (!should_stop(requestQuit) && boosted_total_trials < boosted_max_trials) {
                    ++boosted_total_trials;

                    const size_t old_max_candidates = params_.max_candidate_points;
                    const size_t old_radial_layers = params_.radial_layers;
                    const size_t old_angles = params_.angles_per_layer;

                    params_.max_candidate_points = std::min<size_t>(std::max<size_t>(old_max_candidates, old_max_candidates * 2), 40000);
                    params_.radial_layers = std::min<size_t>(std::max<size_t>(old_radial_layers, old_radial_layers + 16), 160);
                    params_.angles_per_layer = std::min<size_t>(std::max<size_t>(old_angles, old_angles + 32), 256);

                    bool placed_boost = place_shape_with_nfp(shape_idx, placed_indices, /*try_all_rotations=*/true, requestQuit);

                    params_.max_candidate_points = old_max_candidates;
                    params_.radial_layers = old_radial_layers;
                    params_.angles_per_layer = old_angles;

                    if (placed_boost) {
                        placed_indices.push_back(shape_idx);
                        placed = true;
                    }
                }
            }

            if (!placed && !should_stop(requestQuit) && lns_total_trials < lns_max_trials && placed_indices.size() >= 10) {
                auto placed_before = placed_indices;
                auto& target_shape = layout_.sheet_parts[0][shape_idx];
                const auto old_target_rot = target_shape.get_rotation();
                const auto old_target_x = target_shape.get_translate_ft_x();
                const auto old_target_y = target_shape.get_translate_ft_y();

                target_shape.update();
                auto tbbox = target_shape.transformed.bbox();
                double t_w = safe_to_double(tbbox.xmax() - tbbox.xmin());
                double t_h = safe_to_double(tbbox.ymax() - tbbox.ymin());
                double anchor_x = radius - t_w / 2.0;
                double anchor_y = radius - t_h / 2.0;

                auto shape_area = [&](size_t idx) -> double {
                    try {
                        return safe_to_double(geo::pwh_area(*layout_.sheet_parts[0][idx].base));
                    }
                    catch (...) {
                        return 0.0;
                    }
                    };
                auto shape_center = [&](size_t idx) -> std::pair<double, double> {
                    try {
                        auto& s = layout_.sheet_parts[0][idx];
                        s.update();
                        auto bb = s.transformed.bbox();
                        double cx = 0.5 * (safe_to_double(bb.xmin()) + safe_to_double(bb.xmax()));
                        double cy = 0.5 * (safe_to_double(bb.ymin()) + safe_to_double(bb.ymax()));
                        return { cx, cy };
                    }
                    catch (...) {
                        return { 0.0, 0.0 };
                    }
                    };

                std::vector<size_t> near;
                near.reserve(placed_indices.size());
                for (size_t pidx : placed_indices) {
                    near.push_back(pidx);
                }
                std::sort(near.begin(), near.end(), [&](size_t a, size_t b) {
                    auto ca = shape_center(a);
                    auto cb = shape_center(b);
                    double dax = ca.first - anchor_x;
                    double day = ca.second - anchor_y;
                    double dbx = cb.first - anchor_x;
                    double dby = cb.second - anchor_y;
                    return (dax * dax + day * day) < (dbx * dbx + dby * dby);
                    });
                if (near.size() > 12) {
                    near.resize(12);
                }
                std::sort(near.begin(), near.end(), [&](size_t a, size_t b) {
                    double aa = shape_area(a);
                    double ab = shape_area(b);
                    if (aa != ab) {
                        return aa < ab;
                    }
                    auto ca = shape_center(a);
                    auto cb = shape_center(b);
                    double dax = ca.first - anchor_x;
                    double day = ca.second - anchor_y;
                    double dbx = cb.first - anchor_x;
                    double dby = cb.second - anchor_y;
                    return (dax * dax + day * day) < (dbx * dbx + dby * dby);
                    });

                bool lns_success = false;
                for (size_t k = 1; k <= 2 && !lns_success; ++k) {
                    if (should_stop(requestQuit) || lns_total_trials >= lns_max_trials) {
                        break;
                    }
                    if (near.size() < k) {
                        break;
                    }
                    ++lns_total_trials;

                    std::vector<size_t> removed;
                    removed.reserve(k);
                    for (size_t ri = 0; ri < k; ++ri) {
                        removed.push_back(near[ri]);
                    }

                    struct SavedState {
                        uint32_t rot;
                        geo::FT x;
                        geo::FT y;
                    };
                    std::vector<std::pair<size_t, SavedState>> removed_state;
                    removed_state.reserve(removed.size());
                    for (size_t ridx : removed) {
                        auto& s = layout_.sheet_parts[0][ridx];
                        removed_state.push_back({ ridx, SavedState{ s.get_rotation(), s.get_translate_ft_x(), s.get_translate_ft_y() } });
                    }

                    auto remove_from_vec = [&](std::vector<size_t>& vec, size_t val) {
                        vec.erase(std::remove(vec.begin(), vec.end(), val), vec.end());
                        };

                    for (size_t ridx : removed) {
                        remove_from_vec(placed_indices, ridx);
                        auto& s = layout_.sheet_parts[0][ridx];
                        s.set_translate(geo::FT(outside_offset), geo::FT(outside_offset));
                        s.update();
                    }

                    bool placed_target = place_shape_with_nfp(shape_idx, placed_indices, /*try_all_rotations=*/true, requestQuit);
                    if (placed_target) {
                        placed_indices.push_back(shape_idx);
                        bool reinsert_ok = true;
                        for (size_t ridx : removed) {
                            if (should_stop(requestQuit)) {
                                reinsert_ok = false;
                                break;
                            }
                            bool placed_back = place_shape_with_nfp(ridx, placed_indices, /*try_all_rotations=*/true, requestQuit);
                            if (!placed_back) {
                                reinsert_ok = false;
                                break;
                            }
                            placed_indices.push_back(ridx);
                        }
                        if (reinsert_ok) {
                            lns_success = true;
                        }
                    }

                    if (!lns_success) {
                        placed_indices = placed_before;
                        target_shape.set_rotation(old_target_rot);
                        target_shape.set_translate(old_target_x, old_target_y);
                        target_shape.update();
                        for (const auto& kv : removed_state) {
                            auto& s = layout_.sheet_parts[0][kv.first];
                            s.set_rotation(kv.second.rot);
                            s.set_translate(kv.second.x, kv.second.y);
                            s.update();
                        }
                    }
                }
            }
            // 如果无法放置，继续尝试下一个零件（不直接返回false）
            // 这样可以让算法尝试放置尽可能多的零件，即使不是所有零件都能放置

            // 每放置5个零件，报告一次进度
            // 或者每尝试10个零件也报告一次（即使没有放置成功），确保进度有更新
            if (progress_callback) {
                bool should_report = false;
                if (placed_indices.size() % 5 == 0 && placed_indices.size() > 0) {
                    should_report = true;
                }
                else if ((i + 1) % 10 == 0) {
                    should_report = true;
                }
                else if (i == indices.size() - 1) {
                    // 最后一个零件，必须报告
                    should_report = true;
                }

                if (should_report) {
                    // 计算当前利用率并更新最佳结果
                    double current_util = calculate_utilization();
                    if (current_util > best_utilization_) {
                        best_utilization_ = current_util;
                        layout_.best_utilization = best_utilization_;
                        layout_.best_result = layout_.sheet_parts;
                    }
                    else if (best_utilization_ > 0) {
                        // 即使当前利用率没有提高，也要确保 best_result 是最新的
                        layout_.best_utilization = best_utilization_;
                        layout_.best_result = layout_.sheet_parts;
                    }
                    (*progress_callback)(layout_);
                }
            }
        }

        if (!placed_indices.empty() && !should_stop(requestQuit)) {
            std::set<size_t> placed_set_tmp(placed_indices.begin(), placed_indices.end());
            for (size_t i = 0; i < indices.size(); ++i) {
                if (should_stop(requestQuit)) {
                    break;
                }
                size_t shape_idx = indices[i];
                if (placed_set_tmp.find(shape_idx) != placed_set_tmp.end()) {
                    continue;
                }
                if (can_never_fit_in_circle(shape_idx)) {
                    continue;
                }
                bool placed = place_shape_with_nfp(shape_idx, placed_indices, /*try_all_rotations=*/true, requestQuit);
                if (!placed && !should_stop(requestQuit) && boosted_total_trials < boosted_max_trials) {
                    ++boosted_total_trials;

                    const size_t old_max_candidates = params_.max_candidate_points;
                    const size_t old_radial_layers = params_.radial_layers;
                    const size_t old_angles = params_.angles_per_layer;

                    params_.max_candidate_points = std::min<size_t>(std::max<size_t>(old_max_candidates, old_max_candidates * 2), 40000);
                    params_.radial_layers = std::min<size_t>(std::max<size_t>(old_radial_layers, old_radial_layers + 16), 160);
                    params_.angles_per_layer = std::min<size_t>(std::max<size_t>(old_angles, old_angles + 32), 256);

                    placed = place_shape_with_nfp(shape_idx, placed_indices, /*try_all_rotations=*/true, requestQuit);

                    params_.max_candidate_points = old_max_candidates;
                    params_.radial_layers = old_radial_layers;
                    params_.angles_per_layer = old_angles;
                }
                if (placed) {
                    placed_indices.push_back(shape_idx);
                    placed_set_tmp.insert(shape_idx);
                    if (progress_callback && (placed_indices.size() % 5 == 0)) {
                        layout_.best_result = layout_.sheet_parts;
                        layout_.best_utilization = best_utilization_;
                        (*progress_callback)(layout_);
                    }
                }
            }
        }

        // 放置完成后，将未放置的零件移动到板材外面（避免影响可视化）
        double radius_final = diameter / 2.0;
        double outside_offset_final = -radius_final * 2.5; // 放在板材外足够远的位置
        std::set<size_t> placed_set(placed_indices.begin(), placed_indices.end());
        for (size_t i = 0; i < layout_.poly_num; ++i) {
            if (placed_set.find(i) == placed_set.end()) {
                // 未放置的零件，移动到板材外面
                auto& shape = layout_.sheet_parts[0][i];
                // 使用不同的偏移，避免重叠显示
                double offset_x = outside_offset_final + i * radius_final * 0.1;
                double offset_y = outside_offset_final;
                shape.set_translate(geo::FT(offset_x), geo::FT(offset_y));
                shape.update();
            }
        }

        // 放置完成后，更新最佳结果，并打印一次调试信息
        double final_util = 0.0;
        try {
            final_util = calculate_utilization();
        }
        catch (...) {
            final_util = 0.0;
        }

        qDebug() << "[DEBUG] try_place_all_parts: 结束, diameter=" << diameter
            << ", placed_count=" << placed_indices.size()
            << ", total_parts=" << layout_.poly_num
            << ", utilization=" << final_util;

        if (final_util > best_utilization_) {
            best_utilization_ = final_util;
            layout_.best_utilization = best_utilization_;
            layout_.best_result = layout_.sheet_parts;

            if (progress_callback) {
                (*progress_callback)(layout_);
            }
        }

        // 如果成功放置了至少一个零件，返回true
        // 这样即使不是所有零件都能放置，也能继续优化已放置的零件
        return !placed_indices.empty();
    }

    void CircleNesting::compact_layout(size_t iterations, volatile bool* requestQuit,
        std::function<void(const Layout&)>* progress_callback) {
        if (layout_.sheet_parts.empty() || layout_.sheet_parts[0].empty()) {
            return;
        }

        // 缓存常用值，避免重复计算
        const double radius = safe_to_double(layout_.sheets[0].diameter) / 2.0;
        const double center_x = radius;
        const double center_y = radius;
        const double step_size = radius * params_.compact_step_size;

        // 模拟退火参数
        double temperature = params_.sa_initial_temp;
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<double> dis(0.0, 1.0);

        double current_utilization = calculate_utilization();
        double best_utilization_this_compact = current_utilization;

        // 预分配，循环内复用
        std::vector<size_t> indices(layout_.poly_num);
        std::iota(indices.begin(), indices.end(), 0);

        std::vector<size_t> other_indices;
        other_indices.reserve(layout_.poly_num);

        std::vector<std::pair<double, double>> directions;
        directions.reserve(4);

        bool should_quit = false;

        size_t no_improve_iters = 0;

        for (size_t iter = 0; iter < iterations; ++iter) {
            if (should_stop(requestQuit)) {
                should_quit = true;
                break;
            }

            bool improved = false;

            // 随机打乱顺序
            std::shuffle(indices.begin(), indices.end(), gen);

            for (size_t idx : indices) {
                if (should_stop(requestQuit)) {
                    should_quit = true;
                    break;
                }
                auto& shape = layout_.sheet_parts[0][idx];

                // ============================================================
                // 额外一步：使用 NFP 进行局部“贴合重定位”（图形匹配式局部优化）
                // 偶尔将当前零件从当前位置拿起，重新通过 NFP 候选点寻找更紧凑的位置，
                // 相当于对该零件做一次“局部重新排布”。
                // ============================================================
                if (!should_quit && params_.prioritize_compact_positions && iter % 20 == 0) {
                    // 构造"其他零件"集合（复用预分配的容器）
                    other_indices.clear();
                    for (size_t j = 0; j < layout_.poly_num; ++j) {
                        if (j != idx) other_indices.push_back(j);
                    }

                    auto old_x = shape.get_translate_ft_x();
                    auto old_y = shape.get_translate_ft_y();
                    uint32_t old_rot = shape.get_rotation();

                    // 使用现有 NFP 放置逻辑，对该零件做一次局部重定位
                    bool placed_better = place_shape_with_nfp(idx, other_indices, params_.try_all_rotations);
                    if (placed_better && is_valid_placement(idx, other_indices)) {
                        double new_util = calculate_utilization();
                        double delta = new_util - current_utilization;

                        // 接受改进或按概率接受次优解（模拟退火）
                        bool accept = false;
                        if (delta > 0) {
                            accept = true;
                            improved = true;
                            current_utilization = new_util;
                            if (new_util > best_utilization_this_compact) {
                                best_utilization_this_compact = new_util;
                            }
                        }
                        else if (params_.use_simulated_annealing && temperature > 0) {
                            double prob = std::exp(delta / temperature);
                            if (dis(gen) < prob) {
                                accept = true;
                            }
                        }

                        if (!accept) {
                            shape.set_rotation(old_rot);
                            shape.set_translate(old_x, old_y);
                            shape.update();
                        }
                    }
                    else {
                        shape.set_rotation(old_rot);
                        shape.set_translate(old_x, old_y);
                        shape.update();
                    }
                }

                // 多方向搜索：向中心、向最近零件、随机方向（复用预分配的容器）
                directions.clear();

                // 方向1：向中心
                auto bbox = shape.transformed.bbox();
                double cx = safe_to_double(bbox.xmin() + bbox.xmax()) / 2.0;
                double cy = safe_to_double(bbox.ymin() + bbox.ymax()) / 2.0;
                double dx_center = center_x - cx;
                double dy_center = center_y - cy;
                double dist_center = std::sqrt(dx_center * dx_center + dy_center * dy_center);
                if (dist_center > 1e-6) {
                    directions.push_back({ dx_center / dist_center, dy_center / dist_center });
                }

                // 方向2：向最近已放置零件（填充空隙）
                double min_dist = std::numeric_limits<double>::max();
                double dx_to_part = 0, dy_to_part = 0;
                for (size_t j = 0; j < layout_.poly_num; ++j) {
                    if (j == idx) continue;
                    auto bbox_j = layout_.sheet_parts[0][j].transformed.bbox();
                    double cx_j = safe_to_double(bbox_j.xmin() + bbox_j.xmax()) / 2.0;
                    double cy_j = safe_to_double(bbox_j.ymin() + bbox_j.ymax()) / 2.0;
                    double dist = std::sqrt((cx - cx_j) * (cx - cx_j) + (cy - cy_j) * (cy - cy_j));
                    if (dist < min_dist && dist > 1e-6) {
                        min_dist = dist;
                        dx_to_part = cx_j - cx;
                        dy_to_part = cy_j - cy;
                        double norm = std::sqrt(dx_to_part * dx_to_part + dy_to_part * dy_to_part);
                        if (norm > 1e-6) {
                            dx_to_part /= norm;
                            dy_to_part /= norm;
                        }
                    }
                }
                if (min_dist < std::numeric_limits<double>::max()) {
                    directions.push_back({ dx_to_part, dy_to_part });
                }

                // 方向3：向最大废料区域（如果启用废料最小化）
                if (params_.use_waste_minimization && iter % 10 == 0) {
                    try {
                        std::vector<size_t> other_indices_for_waste;
                        for (size_t j = 0; j < layout_.poly_num; ++j) {
                            if (j != idx) other_indices_for_waste.push_back(j);
                        }
                        auto waste_regions = calculate_waste_regions(other_indices_for_waste);
                        if (!waste_regions.empty()) {
                            // 找到最大废料区域的质心
                            const auto& largest_waste = waste_regions[0];
                            try {
                                double waste_cx = 0.0, waste_cy = 0.0;
                                size_t vertex_count = 0;
                                for (auto v = largest_waste.outer_boundary().vertices_begin();
                                    v != largest_waste.outer_boundary().vertices_end(); ++v) {
                                    waste_cx += safe_to_double(v->x());
                                    waste_cy += safe_to_double(v->y());
                                    vertex_count++;
                                }
                                if (vertex_count > 0) {
                                    waste_cx /= vertex_count;
                                    waste_cy /= vertex_count;
                                    double dx_to_waste = waste_cx - cx;
                                    double dy_to_waste = waste_cy - cy;
                                    double dist_to_waste = std::sqrt(dx_to_waste * dx_to_waste + dy_to_waste * dy_to_waste);
                                    if (dist_to_waste > 1e-6) {
                                        directions.push_back({ dx_to_waste / dist_to_waste, dy_to_waste / dist_to_waste });
                                    }
                                }
                            }
                            catch (...) {
                                // 如果计算失败，跳过
                            }
                        }
                    }
                    catch (...) {
                        // 如果废料计算失败，跳过
                    }
                }

                // 方向4：随机方向（探索性搜索）
                double random_angle = dis(gen) * 2.0 * Sheet::kPi;
                directions.push_back({ std::cos(random_angle), std::sin(random_angle) });

                // 尝试每个方向
                for (const auto& dir : directions) {
                    if (should_stop(requestQuit)) {
                        should_quit = true;
                        break;
                    }
                    double current_x = shape.get_translate_double_x();
                    double current_y = shape.get_translate_double_y();

                    // 尝试移动（可能接受次优解，模拟退火）
                    double move_x = dir.first * step_size;
                    double move_y = dir.second * step_size;
                    double new_x = current_x + move_x;
                    double new_y = current_y + move_y;

                    auto old_x = shape.get_translate_ft_x();
                    auto old_y = shape.get_translate_ft_y();

                    shape.set_translate(geo::FT(new_x), geo::FT(new_y));
                    shape.update();

                    // 严格检查：确保移动后仍然遵守两个约束（刀头不能超出板材，刀头之间不能重叠）
                    std::vector<size_t> other_indices;
                    for (size_t j = 0; j < layout_.poly_num; ++j) {
                        if (j != idx) {
                            other_indices.push_back(j);
                        }
                    }

                    if (!should_quit && is_valid_placement(idx, other_indices)) {
                        double new_utilization = calculate_utilization();
                        double delta = new_utilization - current_utilization;

                        // 接受改进或按概率接受次优解（模拟退火）
                        bool accept = false;
                        if (delta > 0) {
                            accept = true;
                            improved = true;
                            current_utilization = new_utilization;
                            if (new_utilization > best_utilization_this_compact) {
                                best_utilization_this_compact = new_utilization;
                            }
                        }
                        else if (params_.use_simulated_annealing && temperature > 0) {
                            double prob = std::exp(delta / temperature);
                            if (dis(gen) < prob) {
                                accept = true;
                            }
                        }

                        if (!accept) {
                            // 恢复
                            shape.set_translate(old_x, old_y);
                            shape.update();
                        }
                    }
                    else {
                        // 恢复
                        shape.set_translate(old_x, old_y);
                        shape.update();
                    }
                }
            }

            if (should_quit) {
                break;
            }

            if (improved) {
                no_improve_iters = 0;
            }
            else {
                ++no_improve_iters;
            }

            // 降低温度（模拟退火）
            if (params_.use_simulated_annealing) {
                temperature *= params_.sa_cooling_rate;
            }

            if (params_.use_simulated_annealing) {
                if (temperature < 1e-6) {
                    temperature = 1e-6;
                }
                if (no_improve_iters > 250) {
                    const double reheat = std::max(1e-6, params_.sa_initial_temp * 0.25);
                    if (temperature < reheat) {
                        temperature = reheat;
                    }
                    no_improve_iters = 0;
                }
            }

            // 如果改进了利用率，更新最佳结果
            if (improved && best_utilization_this_compact > best_utilization_) {
                best_utilization_ = best_utilization_this_compact;
                layout_.best_utilization = best_utilization_;
                layout_.best_result = layout_.sheet_parts;
            }

            // 每10次迭代报告一次进度（只在有改进时更新best_result，确保显示的是最佳值）
            if (progress_callback && iter % 10 == 0) {
                // 确保 best_result 是最新的最佳状态
                if (best_utilization_this_compact > best_utilization_) {
                    best_utilization_ = best_utilization_this_compact;
                    layout_.best_utilization = best_utilization_;
                    layout_.best_result = layout_.sheet_parts;
                }
                (*progress_callback)(layout_);
            }

            if (!improved && iter > 50 && !params_.use_simulated_annealing) {
                // 如果没有改进且已经迭代多次，提前退出（模拟退火模式下继续）
                break;
            }
        }

        // 紧凑化结束后，确保更新最佳结果（如果中途退出，也尽量用当前状态）
        double final_utilization = calculate_utilization();
        if (final_utilization > best_utilization_) {
            best_utilization_ = final_utilization;
            layout_.best_utilization = best_utilization_;
            layout_.best_result = layout_.sheet_parts;

            if (progress_callback) {
                (*progress_callback)(layout_);
            }
        }
    }

    void CircleNesting::local_flip_micro_shift_repair(volatile bool* requestQuit,
        std::function<void(const Layout&)>* progress_callback) {
        if (layout_.sheet_parts.empty() || layout_.sheet_parts[0].empty()) {
            qWarning() << "[LOCAL_REPAIR] skip: empty parts";
            return;
        }

        const double radius = safe_to_double(layout_.sheets[0].diameter) / 2.0;
        const double center_x = radius;
        const double center_y = radius;
        const double radius_sq = radius * radius;
        const double compactness_eps = std::max(1e-9, radius * 0.002);

        qWarning() << "[LOCAL_REPAIR] begin: poly_num=" << layout_.poly_num
            << ", radius=" << radius
            << ", eps=" << compactness_eps;

        std::vector<std::pair<double, size_t>> small_candidates;
        small_candidates.reserve(layout_.poly_num);
        for (size_t i = 0; i < layout_.poly_num; ++i) {
            if (should_stop(requestQuit)) {
                qWarning() << "[LOCAL_REPAIR] abort: should_stop during candidate scan";
                return;
            }
            try {
                auto bbox = layout_.sheet_parts[0][i].transformed.bbox();
                double cx = safe_to_double(bbox.xmin() + bbox.xmax()) / 2.0;
                double cy = safe_to_double(bbox.ymin() + bbox.ymax()) / 2.0;
                double dist_sq = (cx - center_x) * (cx - center_x) + (cy - center_y) * (cy - center_y);
                if (dist_sq >= radius_sq * 1.21) {
                    continue;
                }
                double w = safe_to_double(bbox.xmax() - bbox.xmin());
                double h = safe_to_double(bbox.ymax() - bbox.ymin());
                double area_proxy = std::max(0.0, w * h);
                small_candidates.push_back({ area_proxy, i });
            }catch (...) {
            }
        }

        if (small_candidates.size() < 2) {
            qWarning() << "[LOCAL_REPAIR] skip: small_candidates<2";
            return;
        }

        std::sort(small_candidates.begin(), small_candidates.end(),
            [](const auto& a, const auto& b) { return a.first < b.first; });

        std::vector<size_t> active;
        active.reserve(small_candidates.size());
        size_t pick = static_cast<size_t>(std::ceil(static_cast<double>(small_candidates.size()) * 0.2));
        pick = std::max<size_t>(pick, 30);
        pick = std::min<size_t>(pick, 200);
        pick = std::min<size_t>(pick, small_candidates.size());
        for (size_t k = 0; k < pick; ++k) {
            active.push_back(small_candidates[k].second);
        }

        qWarning() << "[LOCAL_REPAIR] picked=" << active.size() << " of small_candidates=" << small_candidates.size();

        std::random_device rd;
        std::mt19937 gen(rd());
        std::shuffle(active.begin(), active.end(), gen);

        const size_t max_shapes = std::min<size_t>(active.size(), 200);
        const size_t max_candidates_per_rot = 300;

        auto compactness = [&]() -> double {
            double max_r = 0.0;
            try {
                for (const auto& s : layout_.sheet_parts[0]) {
                    auto bb = s.transformed.bbox();
                    double xs[2] = { safe_to_double(bb.xmin()), safe_to_double(bb.xmax()) };
                    double ys[2] = { safe_to_double(bb.ymin()), safe_to_double(bb.ymax()) };
                    for (double x : xs) {
                        for (double y : ys) {
                            double dx = x - center_x;
                            double dy = y - center_y;
                            double r = std::sqrt(dx * dx + dy * dy);
                            if (r > max_r) max_r = r;
                        }
                    }
                }
            }catch (...) {
            }
            return max_r;
            };

        auto local_score = [&](size_t idx, const std::vector<size_t>& others) -> double {
            try {
                const auto& shape = layout_.sheet_parts[0][idx];
                auto bbox = shape.transformed.bbox();
                double cx = safe_to_double(bbox.xmin() + bbox.xmax()) / 2.0;
                double cy = safe_to_double(bbox.ymin() + bbox.ymax()) / 2.0;
                double dx = cx - center_x;
                double dy = cy - center_y;
                double dist_sq = dx * dx + dy * dy;

                double min_dist = std::numeric_limits<double>::max();
                double nearest_same_dist = std::numeric_limits<double>::max();
                uint32_t nearest_same_rotation = 0;
                for (size_t j : others) {
                    const auto& s = layout_.sheet_parts[0][j];
                    auto bb = s.transformed.bbox();
                    double cxx = safe_to_double(bb.xmin() + bb.xmax()) / 2.0;
                    double cyy = safe_to_double(bb.ymin() + bb.ymax()) / 2.0;
                    double d = std::sqrt((cx - cxx) * (cx - cxx) + (cy - cyy) * (cy - cyy));
                    min_dist = std::min(min_dist, d);
                    if (s.item_idx == shape.item_idx) {
                        if (d < nearest_same_dist) {
                            nearest_same_dist = d;
                            nearest_same_rotation = s.get_rotation();
                        }
                    }
                }

                double score = dist_sq * 0.7 + min_dist * 0.3;
                const uint32_t allowed = std::max<uint32_t>(1, shape.allowed_rotations);
                if (allowed >= 2) {
                    double neighbor_threshold = 0.0;
                    try {
                        double w = safe_to_double(bbox.xmax() - bbox.xmin());
                        double h = safe_to_double(bbox.ymax() - bbox.ymin());
                        double diag = std::sqrt(w * w + h * h);
                        neighbor_threshold = std::max(1e-9, diag * 3.5);
                    }
                    catch (...) {
                        neighbor_threshold = 0.0;
                    }

                    if (neighbor_threshold > 0.0 && nearest_same_dist < neighbor_threshold) {
                        uint32_t desired = (nearest_same_rotation + allowed / 2) % allowed;
                        if ((shape.get_rotation() % allowed) != desired) {
                            score += 2e-3 * (radius * radius + 1.0);
                        }
                    }
                }

                return score;
            }
            catch (...) {
                return std::numeric_limits<double>::max();
            }
            };

        auto center_of = [&](size_t idx) -> std::pair<double, double> {
            try {
                const auto& s = layout_.sheet_parts[0][idx];
                auto bb = s.transformed.bbox();
                double cx = safe_to_double(bb.xmin() + bb.xmax()) / 2.0;
                double cy = safe_to_double(bb.ymin() + bb.ymax()) / 2.0;
                return { cx, cy };
            }
            catch (...) {
                return { 0.0, 0.0 };
            }
            };

        auto nearest_same_dist = [&](size_t idx, const std::vector<size_t>& others,
            std::pair<double, double>* dir_unit) -> double {
                double best = std::numeric_limits<double>::max();
                try {
                    const auto& s0 = layout_.sheet_parts[0][idx];
                    auto c0 = center_of(idx);
                    double best_dx = 0.0;
                    double best_dy = 0.0;
                    uint32_t best_rot = 0;
                    for (size_t j : others) {
                        const auto& s = layout_.sheet_parts[0][j];
                        if (s.item_idx != s0.item_idx) {
                            continue;
                        }
                        auto cj = center_of(j);
                        double dx = cj.first - c0.first;
                        double dy = cj.second - c0.second;
                        double d = std::sqrt(dx * dx + dy * dy);
                        if (d < best) {
                            best = d;
                            best_dx = dx;
                            best_dy = dy;
                            best_rot = s.get_rotation();
                        }
                    }
                    if (dir_unit) {
                        if (best < std::numeric_limits<double>::max() && best > 1e-12) {
                            (*dir_unit) = { best_dx / best, best_dy / best };
                        }
                        else {
                            (*dir_unit) = { 0.0, 0.0 };
                        }
                    }
                }
                catch (...) {
                    if (dir_unit) {
                        (*dir_unit) = { 0.0, 0.0 };
                    }
                }
                return best;
            };

        double comp_initial = compactness();
        double comp_current = comp_initial;

        size_t accepted = 0;
        for (size_t k = 0; k < max_shapes; ++k) {
            if (should_stop(requestQuit)) {
                qWarning() << "[LOCAL_REPAIR] abort: should_stop during search";
                break;
            }

            size_t idx = active[k];
            auto& shape = layout_.sheet_parts[0][idx];
            const uint32_t allowed = std::max<uint32_t>(1, shape.allowed_rotations);
            if (allowed < 2) {
                continue;
            }

            std::vector<size_t> others;
            others.reserve(layout_.poly_num > 0 ? (layout_.poly_num - 1) : 0);
            for (size_t j = 0; j < layout_.poly_num; ++j) {
                if (j != idx) {
                    others.push_back(j);
                }
            }

            const auto old_x = shape.get_translate_ft_x();
            const auto old_y = shape.get_translate_ft_y();
            const uint32_t old_rot = shape.get_rotation();

            double comp_before_move = comp_current;

            double score_before = local_score(idx, others);
            std::pair<double, double> dir_unit{ 0.0, 0.0 };
            double same_before = nearest_same_dist(idx, others, &dir_unit);

            double best_score = score_before;
            double best_same = same_before;
            double best_comp = comp_before_move;
            geo::FT best_x = old_x;
            geo::FT best_y = old_y;
            uint32_t best_rot = old_rot;

            uint32_t rot_a = old_rot % allowed;
            uint32_t rot_b = (rot_a + allowed / 2) % allowed;

            for (uint32_t rot_try : { rot_b, rot_a }) {
                if (should_stop(requestQuit)) {
                    break;
                }
                shape.set_rotation(rot_try);
                shape.update();

                auto ifr = geo::comp_ifr(layout_.sheets[0].sheet, shape.transformed);
                if (ifr.is_empty()) {
                    continue;
                }

                std::vector<geo::Point_2> candidates_for_rot;
                try {
                    candidates_for_rot = generate_circular_candidates(idx, others, ifr);
                }
                catch (...) {
                    candidates_for_rot.clear();
                }

                // 邻居方向/垂直方向微网格候选点：用于打破“同向畴”，促进正反咬合
                try {
                    const double r = radius;
                    const double cx0 = r;
                    const double cy0 = r;
                    auto bbox_r = shape.transformed.bbox();
                    double w = safe_to_double(bbox_r.xmax() - bbox_r.xmin());
                    double h = safe_to_double(bbox_r.ymax() - bbox_r.ymin());
                    double diag = std::sqrt(w * w + h * h);
                    double step = std::max(1e-9, std::min(diag * 0.18, r * 0.03));
                    double step2 = step * 0.55;

                    const double ox = safe_to_double(shape.get_translate_ft_x());
                    const double oy = safe_to_double(shape.get_translate_ft_y());

                    double ux = dir_unit.first;
                    double uy = dir_unit.second;
                    double px = -uy;
                    double py = ux;

                    auto try_push = [&](double x, double y) {
                        geo::Point_2 p{ geo::FT(x), geo::FT(y) };
                        // IFR 过滤
                        try {
                            if (ifr.bounded_side(p) == CGAL::ON_UNBOUNDED_SIDE) {
                                return;
                            }
                        }
                        catch (...) {
                            // 若 bounded_side 失败则不添加
                            return;
                        }
                        // 圆内过滤（用中心点近似）
                        double dx = x - cx0;
                        double dy = y - cy0;
                        if (dx * dx + dy * dy > r * r) {
                            return;
                        }
                        candidates_for_rot.push_back(p);
                        };

                    // 沿邻居方向推进/后退 + 垂直微移（形成“咬合”常见形态）
                    const double k1[2] = { step, step2 };
                    for (double s : k1) {
                        try_push(ox + ux * s, oy + uy * s);
                        try_push(ox - ux * s, oy - uy * s);
                        try_push(ox + px * s, oy + py * s);
                        try_push(ox - px * s, oy - py * s);

                        try_push(ox + ux * s + px * step2, oy + uy * s + py * step2);
                        try_push(ox + ux * s - px * step2, oy + uy * s - py * step2);
                        try_push(ox - ux * s + px * step2, oy - uy * s + py * step2);
                        try_push(ox - ux * s - px * step2, oy - uy * s - py * step2);
                    }
                }catch (...) {
                }

                if (candidates_for_rot.empty()) {
                    continue;
                }

                {
                    const double ox = safe_to_double(old_x);
                    const double oy = safe_to_double(old_y);
                    const auto c0 = center_of(idx);
                    const double step_to_neighbor = (same_before < std::numeric_limits<double>::max()) ?
                        std::min(same_before, radius * 0.5) : 0.0;
                    const double tx = c0.first + dir_unit.first * step_to_neighbor;
                    const double ty = c0.second + dir_unit.second * step_to_neighbor;

                    std::sort(candidates_for_rot.begin(), candidates_for_rot.end(),
                        [&](const geo::Point_2& a, const geo::Point_2& b) {
                            double ax = safe_to_double(a.x());
                            double ay = safe_to_double(a.y());
                            double bx = safe_to_double(b.x());
                            double by = safe_to_double(b.y());

                            double da0x = ax - ox;
                            double da0y = ay - oy;
                            double db0x = bx - ox;
                            double db0y = by - oy;
                            double d0a = da0x * da0x + da0y * da0y;
                            double d0b = db0x * db0x + db0y * db0y;

                            double datx = ax - tx;
                            double daty = ay - ty;
                            double dbtx = bx - tx;
                            double dbty = by - ty;
                            double dta = datx * datx + daty * daty;
                            double dtb = dbtx * dbtx + dbty * dbty;

                            return (d0a * 0.7 + dta * 0.3) < (d0b * 0.7 + dtb * 0.3);
                        });
                }

                const size_t limit = std::min(max_candidates_per_rot, candidates_for_rot.size());
                for (size_t ci = 0; ci < limit; ++ci) {
                    if (should_stop(requestQuit)) {
                        break;
                    }

                    shape.set_translate(candidates_for_rot[ci].x(), candidates_for_rot[ci].y());
                    shape.update();

                    if (!is_valid_placement(idx, others)) {
                        continue;
                    }

                    double comp_try = compactness();
                    if (comp_try > comp_before_move + compactness_eps * 2.0) {
                        continue;
                    }

                    double score_try = local_score(idx, others);
                    double same_try = nearest_same_dist(idx, others, nullptr);

                    bool better = false;
                    if (same_try + 1e-9 < best_same) {
                        better = true;
                    }
                    else if (score_try + 1e-12 < best_score) {
                        better = true;
                    }
                    else if (comp_try + 1e-9 < best_comp) {
                        better = true;
                    }

                    if (better) {
                        best_comp = comp_try;
                        best_score = score_try;
                        best_same = same_try;
                        best_rot = rot_try;
                        best_x = shape.get_translate_ft_x();
                        best_y = shape.get_translate_ft_y();
                    }
                }
            }

            shape.set_rotation(best_rot);
            shape.set_translate(best_x, best_y);
            shape.update();

            if (!is_valid_placement(idx, others)) {
                shape.set_rotation(old_rot);
                shape.set_translate(old_x, old_y);
                shape.update();
                continue;
            }

            if (best_rot == old_rot && best_x == old_x && best_y == old_y) {
                continue;
            }

            double comp_after_move = compactness();
            double score_after = local_score(idx, others);
            double same_after = nearest_same_dist(idx, others, nullptr);
            if (comp_after_move > comp_before_move + compactness_eps ||
                !((same_after + 1e-9 < same_before) || (score_after + 1e-12 < score_before))) {
                shape.set_rotation(old_rot);
                shape.set_translate(old_x, old_y);
                shape.update();
                continue;
            }

            comp_current = comp_after_move;

            ++accepted;
            if (accepted % 25 == 0 && progress_callback) {
                double util_now = 0.0;
                try {
                    util_now = calculate_utilization();
                }
                catch (...) {
                    util_now = 0.0;
                }
                if (util_now > best_utilization_) {
                    best_utilization_ = util_now;
                    layout_.best_utilization = best_utilization_;
                    layout_.best_result = layout_.sheet_parts;
                }
                (*progress_callback)(layout_);
            }
        }

        if (progress_callback && comp_current + 1e-9 < comp_initial) {
            (*progress_callback)(layout_);
        }

        qWarning() << "[LOCAL_REPAIR] end: tried=" << max_shapes
            << ", accepted=" << accepted
            << ", compactness=" << comp_initial << "->" << comp_current;
    }

    bool CircleNesting::optimize_array_mode(double target_utilization, volatile bool* requestQuit,
        std::function<void(const Layout&)> progress_callback) {
        double diameter = safe_to_double(layout_.sheets[0].diameter);
        if (diameter <= 0.0) {
            return false;
        }

        const double prev_best_util = best_utilization_;
        const auto prev_best_result = layout_.best_result;

        bool placed_any = try_place_all_parts(diameter, requestQuit, progress_callback ? &progress_callback : nullptr);
        if (!placed_any) {
            layout_.best_utilization = 0.0;
            layout_.best_result = layout_.sheet_parts;
            if (progress_callback) {
                progress_callback(layout_);
            }
            return false;
        }

        if (!should_stop(requestQuit)) {
            compact_layout(params_.compact_iterations, requestQuit, progress_callback ? &progress_callback : nullptr);
            local_flip_micro_shift_repair(requestQuit, progress_callback ? &progress_callback : nullptr);
        }

        double util = 0.0;
        try {
            util = calculate_utilization();
        }
        catch (...) {
            util = 0.0;
        }

        const double util_eps = 1e-9;

        if (util > best_utilization_) {
            best_utilization_ = util;
            layout_.best_utilization = best_utilization_;
            layout_.best_result = layout_.sheet_parts;
        }
        else {
            layout_.best_utilization = std::max(prev_best_util, best_utilization_);
            if (util + util_eps >= layout_.best_utilization) {
                layout_.best_result = layout_.sheet_parts;
            }
            else if (!prev_best_result.empty()) {
                layout_.best_result = prev_best_result;
                layout_.sheet_parts = layout_.best_result;
            }
        }

        if (progress_callback) {
            progress_callback(layout_);
        }

        return best_utilization_ >= target_utilization;
    }

    bool CircleNesting::optimize_diameter(double target_utilization, volatile bool* requestQuit,
        std::function<void(const Layout&)> progress_callback) {
        double theoretical_min = calculate_theoretical_min_diameter();
        double current_diameter = safe_to_double(layout_.sheets[0].diameter);
        if (current_diameter <= 0.0) {
            return false;
        }

        const double prev_best_util = best_utilization_;
        const auto prev_best_result = layout_.best_result;

        double min_diameter = theoretical_min * params_.min_diameter_ratio;
        if (min_diameter <= 0.0) {
            min_diameter = theoretical_min;
        }
        if (min_diameter > current_diameter) {
            min_diameter = current_diameter;
        }

        double best_diameter = current_diameter;
        if (params_.use_binary_search) {
            best_diameter = binary_search_diameter(min_diameter, current_diameter,
                target_utilization, requestQuit, progress_callback);
        }

        std::vector<std::vector<TransformedShape>> best_layout =
            (!layout_.best_result.empty()) ? layout_.best_result : layout_.sheet_parts;
        double best_util_at_d = (!layout_.best_result.empty()) ? layout_.best_utilization : best_utilization_;

        auto all_parts_inside_circle_for = [&]() -> bool {
            if (layout_.sheet_parts.empty() || layout_.sheet_parts[0].empty()) {
                return true;
            }
            double radius = safe_to_double(layout_.sheets[0].diameter) / 2.0;
            double center_x = radius;
            double center_y = radius;
            double radius_sq = radius * radius;
            const double tol = 1e-10;
            for (const auto& s : layout_.sheet_parts[0]) {
                auto txy = s.get_translate_double();
                if (txy.first < 0.0 && txy.second < 0.0) {
                    return false;
                }
                const auto& ob = s.transformed.outer_boundary();
                for (auto vit = ob.vertices_begin(); vit != ob.vertices_end(); ++vit) {
                    double x = safe_to_double(vit->x());
                    double y = safe_to_double(vit->y());
                    double dx = x - center_x;
                    double dy = y - center_y;
                    double d2 = dx * dx + dy * dy;
                    if (d2 > radius_sq * (1.0 + tol)) {
                        return false;
                    }
                }
            }
            return true;
            };

        const size_t refine_rounds = std::min<size_t>(30, std::max<size_t>(3, params_.diameter_optimization_iterations / 10));
        const double shrink_ratio = std::min(0.005, std::max(0.001, params_.diameter_step_ratio));
        const size_t pull_iters = std::min<size_t>(200, std::max<size_t>(40, params_.compact_iterations / 25));
        const double util_eps = 1e-12;

        for (size_t round = 0; round < refine_rounds; ++round) {
            if (should_stop(requestQuit)) {
                break;
            }

            layout_.sheet_parts = best_layout;
            update_circle_diameter(best_diameter);

            // 重启调度：由于内部随机扰动，每轮可能得到不同布局
            schedule_best_order(best_diameter, requestQuit, progress_callback ? &progress_callback : nullptr);

            if (should_stop(requestQuit)) {
                break;
            }
            compact_layout(std::max<size_t>(pull_iters, params_.compact_iterations / 8), requestQuit,
                progress_callback ? &progress_callback : nullptr);
            local_flip_micro_shift_repair(requestQuit, progress_callback ? &progress_callback : nullptr);

            double u_now = 0.0;
            try {
                u_now = calculate_utilization();
            }
            catch (...) {
                u_now = 0.0;
            }
            if (u_now > best_util_at_d + 1e-9) {
                best_util_at_d = u_now;
                best_layout = layout_.sheet_parts;
                if (u_now > best_utilization_) {
                    best_utilization_ = u_now;
                    layout_.best_utilization = best_utilization_;
                    layout_.best_result = best_layout;
                }
            }

            // 小步缩径：最多尝试两次，失败立即回滚。
            for (size_t step = 0; step < 2 && !should_stop(requestQuit); ++step) {
                double cand_d = best_diameter * (1.0 - shrink_ratio);
                if (cand_d < min_diameter) {
                    break;
                }
                if (cand_d <= 0.0) {
                    break;
                }

                std::vector<std::vector<TransformedShape>> cand_layout = best_layout;
                double delta = 0.5 * (cand_d - best_diameter);
                if (!cand_layout.empty()) {
                    for (auto& s : cand_layout[0]) {
                        double x = s.get_translate_double_x();
                        double y = s.get_translate_double_y();
                        s.set_translate(geo::FT(x + delta), geo::FT(y + delta));
                        s.update();
                    }
                }

                layout_.sheet_parts = cand_layout;
                update_circle_diameter(cand_d);

                bool ok = all_parts_inside_circle_for();
                if (!ok) {
                    compact_layout(pull_iters, requestQuit, progress_callback ? &progress_callback : nullptr);
                    ok = all_parts_inside_circle_for();
                }
                if (!ok) {
                    layout_.sheet_parts = best_layout;
                    update_circle_diameter(best_diameter);
                    break;
                }

                double u = 0.0;
                try {
                    u = calculate_utilization();
                }
                catch (...) {
                    u = 0.0;
                }

                if (u + util_eps >= best_util_at_d) {
                    best_diameter = cand_d;
                    best_layout = layout_.sheet_parts;
                    best_util_at_d = u;
                    if (u > best_utilization_) {
                        best_utilization_ = u;
                        layout_.best_utilization = best_utilization_;
                        layout_.best_result = best_layout;
                    }
                }
                else {
                    layout_.sheet_parts = best_layout;
                    update_circle_diameter(best_diameter);
                    break;
                }
            }
        }

        layout_.sheet_parts = best_layout;
        update_circle_diameter(best_diameter);

        if (!should_stop(requestQuit)) {
            compact_layout(params_.compact_iterations, requestQuit, progress_callback ? &progress_callback : nullptr);
            local_flip_micro_shift_repair(requestQuit, progress_callback ? &progress_callback : nullptr);
        }

        double util = 0.0;
        try {
            util = calculate_utilization();
        }
        catch (...) {
            util = 0.0;
        }

        if (util > best_utilization_) {
            best_utilization_ = util;
            layout_.best_utilization = best_utilization_;
            layout_.best_result = layout_.sheet_parts;
        }
        else {
            layout_.best_utilization = std::max(prev_best_util, best_utilization_);
            if (!prev_best_result.empty()) {
                layout_.best_result = prev_best_result;
                layout_.sheet_parts = layout_.best_result;
            }
        }

        if (progress_callback) {
            progress_callback(layout_);
        }

        return best_utilization_ >= target_utilization;
    }

    bool CircleNesting::optimize(double target_utilization, volatile bool* requestQuit,
        std::function<void(const Layout&)> progress_callback) {
        best_utilization_ = 0.0;
        have_validated_best_ = false;
        last_validated_best_utilization_ = 0.0;
        last_validated_best_result_.clear();
        next_cgal_validation_time_ = std::chrono::steady_clock::time_point{};

        auto progress_ptr = progress_callback ? &progress_callback : nullptr;
        auto finish_after_first_stage = [&]() -> bool {
            if (!should_stop(requestQuit) && params_.compact_iterations > 0) {
                compact_layout(params_.compact_iterations, requestQuit, progress_ptr);
            }

            if (!should_stop(requestQuit) && !layout_.sheet_parts.empty() && !layout_.sheet_parts[0].empty()) {
                std::vector<size_t> placed;
                placed.reserve(layout_.poly_num);
                std::vector<size_t> unplaced;
                unplaced.reserve(layout_.poly_num);

                double radius = safe_to_double(layout_.sheets[0].diameter) / 2.0;
                double outside_offset = -radius * 2.0;

                for (size_t i = 0; i < layout_.poly_num; ++i) {
                    const auto& s = layout_.sheet_parts[0][i];
                    auto txy = s.get_translate_double();
                    double tx = txy.first;
                    double ty = txy.second;
                    if (tx < 0.0 && ty < 0.0) {
                        unplaced.push_back(i);
                    }
                    else {
                        placed.push_back(i);
                    }
                }

                size_t insert_trials = 0;
                const size_t insert_max_trials = 20;
                for (size_t idx : unplaced) {
                    if (should_stop(requestQuit) || insert_trials >= insert_max_trials) {
                        break;
                    }
                    if (can_never_fit_in_circle(idx)) {
                        continue;
                    }

                    ++insert_trials;
                    bool ok = place_shape_with_nfp(idx, placed, /*try_all_rotations=*/true, requestQuit);
                    if (!ok && insert_trials < insert_max_trials) {
                        const size_t old_max_candidates = params_.max_candidate_points;
                        const size_t old_radial_layers = params_.radial_layers;
                        const size_t old_angles = params_.angles_per_layer;

                        params_.max_candidate_points = std::min<size_t>(std::max<size_t>(old_max_candidates, old_max_candidates * 2), 40000);
                        params_.radial_layers = std::min<size_t>(std::max<size_t>(old_radial_layers, old_radial_layers + 16), 160);
                        params_.angles_per_layer = std::min<size_t>(std::max<size_t>(old_angles, old_angles + 32), 256);

                        ok = place_shape_with_nfp(idx, placed, /*try_all_rotations=*/true, requestQuit);

                        params_.max_candidate_points = old_max_candidates;
                        params_.radial_layers = old_radial_layers;
                        params_.angles_per_layer = old_angles;
                    }

                    if (!ok && insert_trials < insert_max_trials && placed.size() >= 10 && !should_stop(requestQuit)) {
                        auto shape_area = [&](size_t sidx) -> double {
                            try {
                                return safe_to_double(geo::pwh_area(*layout_.sheet_parts[0][sidx].base));
                            }
                            catch (...) {
                                return 0.0;
                            }
                            };
                        auto shape_center = [&](size_t sidx) -> std::pair<double, double> {
                            try {
                                auto& s = layout_.sheet_parts[0][sidx];
                                s.update();
                                auto bb = s.transformed.bbox();
                                double cx = 0.5 * (safe_to_double(bb.xmin()) + safe_to_double(bb.xmax()));
                                double cy = 0.5 * (safe_to_double(bb.ymin()) + safe_to_double(bb.ymax()));
                                return { cx, cy };
                            }
                            catch (...) {
                                return { radius, radius };
                            }
                            };

                        auto& target_shape = layout_.sheet_parts[0][idx];
                        const auto old_target_rot = target_shape.get_rotation();
                        const auto old_target_txy = target_shape.get_translate_ft();

                        std::vector<size_t> near = placed;
                        std::sort(near.begin(), near.end(), [&](size_t a, size_t b) {
                            double aa = shape_area(a);
                            double ab = shape_area(b);
                            if (aa != ab) {
                                return aa < ab;
                            }
                            auto ca = shape_center(a);
                            auto cb = shape_center(b);
                            double dax = ca.first - radius;
                            double day = ca.second - radius;
                            double dbx = cb.first - radius;
                            double dby = cb.second - radius;
                            return (dax * dax + day * day) < (dbx * dbx + dby * dby);
                            });
                        if (near.size() > 40) {
                            near.resize(40);
                        }

                        bool lns_success = false;
                        const size_t lns_trials_max = 8;
                        size_t lns_trials = 0;
                        for (size_t k = 2; k <= 6 && !lns_success; ++k) {
                            if (should_stop(requestQuit) || lns_trials >= lns_trials_max) {
                                break;
                            }
                            if (near.size() < k) {
                                break;
                            }

                            for (size_t start = 0; start + k <= near.size() && !lns_success; start += k) {
                                if (should_stop(requestQuit) || lns_trials >= lns_trials_max) {
                                    break;
                                }
                                ++lns_trials;

                                std::vector<size_t> removed;
                                removed.reserve(k);
                                for (size_t ri = 0; ri < k; ++ri) {
                                    removed.push_back(near[start + ri]);
                                }

                                struct SavedState {
                                    uint32_t rot;
                                    geo::FT x;
                                    geo::FT y;
                                };
                                std::vector<std::pair<size_t, SavedState>> removed_state;
                                removed_state.reserve(removed.size());
                                for (size_t ridx : removed) {
                                    auto& s = layout_.sheet_parts[0][ridx];
                                    removed_state.push_back({ ridx, SavedState{ s.get_rotation(), s.get_translate_ft_x(), s.get_translate_ft_y() } });
                                }

                                auto remove_from_vec = [&](std::vector<size_t>& vec, size_t val) {
                                    vec.erase(std::remove(vec.begin(), vec.end(), val), vec.end());
                                    };

                                for (size_t ridx : removed) {
                                    remove_from_vec(placed, ridx);
                                    auto& s = layout_.sheet_parts[0][ridx];
                                    s.set_translate(geo::FT(outside_offset), geo::FT(outside_offset));
                                    s.update();
                                }

                                bool placed_target = place_shape_with_nfp(idx, placed, /*try_all_rotations=*/true, requestQuit);
                                if (placed_target) {
                                    placed.push_back(idx);
                                    bool reinsert_ok = true;
                                    for (size_t ridx : removed) {
                                        if (should_stop(requestQuit)) {
                                            reinsert_ok = false;
                                            break;
                                        }
                                        bool placed_back = place_shape_with_nfp(ridx, placed, /*try_all_rotations=*/true, requestQuit);
                                        if (!placed_back) {
                                            reinsert_ok = false;
                                            break;
                                        }
                                        placed.push_back(ridx);
                                    }
                                    if (reinsert_ok) {
                                        lns_success = true;
                                    }
                                }

                                if (!lns_success) {
                                    target_shape.set_rotation(old_target_rot);
                                    target_shape.set_translate(old_target_txy.first, old_target_txy.second);
                                    target_shape.update();
                                    for (const auto& kv : removed_state) {
                                        auto& s = layout_.sheet_parts[0][kv.first];
                                        s.set_rotation(kv.second.rot);
                                        s.set_translate(kv.second.x, kv.second.y);
                                        s.update();
                                    }
                                    placed.clear();
                                    placed.reserve(layout_.poly_num);
                                    for (size_t i = 0; i < layout_.poly_num; ++i) {
                                        const auto& s = layout_.sheet_parts[0][i];
                                        auto txy = s.get_translate_double();
                                        if (!(txy.first < 0.0 && txy.second < 0.0)) {
                                            placed.push_back(i);
                                        }
                                    }
                                }
                            }
                        }
                        ok = lns_success;
                    }
                    if (ok) {
                        if (std::find(placed.begin(), placed.end(), idx) == placed.end()) {
                            placed.push_back(idx);
                        }
                    }
                }
            }

            auto all_parts_inside_circle = [&]() -> bool {
                if (layout_.sheet_parts.empty() || layout_.sheet_parts[0].empty()) {
                    return true;
                }
                double radius = safe_to_double(layout_.sheets[0].diameter) / 2.0;
                double center_x = radius;
                double center_y = radius;
                double radius_sq = radius * radius;
                const double tol = 1e-10;
                for (const auto& s : layout_.sheet_parts[0]) {
                    const auto& ob = s.transformed.outer_boundary();
                    for (auto vit = ob.vertices_begin(); vit != ob.vertices_end(); ++vit) {
                        double x = safe_to_double(vit->x());
                        double y = safe_to_double(vit->y());
                        double dx = x - center_x;
                        double dy = y - center_y;
                        double d2 = dx * dx + dy * dy;
                        if (d2 > radius_sq * (1.0 + tol)) {
                            return false;
                        }
                    }
                }
                return true;
                };

            if (!should_stop(requestQuit) && (params_.use_binary_search || params_.diameter_optimization_iterations > 0)) {
                double best_d = safe_to_double(layout_.sheets[0].diameter);
                if (best_d > 0.0 && !layout_.sheet_parts.empty()) {
                    const double shrink_ratio = std::min(0.005, std::max(0.001, params_.diameter_step_ratio));
                    const size_t steps = (params_.compact_iterations <= 80) ? 2 : 4;
                    const size_t pull_iters = std::min<size_t>(12, std::max<size_t>(4, params_.compact_iterations / 40));
                    const double util_eps = 1e-12;

                    std::vector<std::vector<TransformedShape>> best_layout =
                        (!layout_.best_result.empty()) ? layout_.best_result : layout_.sheet_parts;

                    layout_.sheet_parts = best_layout;
                    update_circle_diameter(best_d);

                    for (size_t step = 0; step < steps; ++step) {
                        if (should_stop(requestQuit)) {
                            break;
                        }

                        double cand_d = best_d * (1.0 - shrink_ratio);
                        if (cand_d <= 0.0) {
                            break;
                        }

                        std::vector<std::vector<TransformedShape>> cand_layout = best_layout;
                        double delta = (cand_d * 0.5) - (best_d * 0.5);
                        if (!cand_layout.empty()) {
                            for (auto& s : cand_layout[0]) {
                                double x = s.get_translate_double_x();
                                double y = s.get_translate_double_y();
                                s.set_translate(geo::FT(x + delta), geo::FT(y + delta));
                                s.update();
                            }
                        }

                        layout_.sheet_parts = cand_layout;
                        update_circle_diameter(cand_d);

                        bool ok = all_parts_inside_circle();
                        if (!ok && pull_iters > 0) {
                            compact_layout(pull_iters, requestQuit, progress_ptr);
                            ok = all_parts_inside_circle();
                        }

                        if (!ok) {
                            layout_.sheet_parts = best_layout;
                            update_circle_diameter(best_d);
                            break;
                        }

                        double u = 0.0;
                        try {
                            u = calculate_utilization();
                        }
                        catch (...) {
                            u = 0.0;
                        }

                        if (u + util_eps >= best_utilization_) {
                            best_d = cand_d;
                            best_layout = layout_.sheet_parts;
                            if (u > best_utilization_) {
                                best_utilization_ = u;
                                layout_.best_utilization = best_utilization_;
                                layout_.best_result = best_layout;
                            }
                        }
                        else {
                            layout_.sheet_parts = best_layout;
                            update_circle_diameter(best_d);
                            break;
                        }
                    }

                    layout_.sheet_parts = best_layout;
                    update_circle_diameter(best_d);
                    if (best_utilization_ > 0.0) {
                        layout_.best_utilization = best_utilization_;
                        layout_.best_result = best_layout;
                    }
                }
            }

            double util_now = 0.0;
            try {
                util_now = calculate_utilization();
            }
            catch (...) {
                util_now = 0.0;
            }
            if (util_now > best_utilization_) {
                best_utilization_ = util_now;
                layout_.best_utilization = best_utilization_;
                layout_.best_result = layout_.sheet_parts;
            }
            else {
                layout_.best_utilization = best_utilization_;
                if (!layout_.best_result.empty() && util_now + 1e-12 < best_utilization_) {
                    layout_.sheet_parts = layout_.best_result;
                }
            }

            maybe_cgal_validate_and_checkpoint_best(requestQuit);
            if (progress_callback) {
                progress_callback(layout_);
            }
            return best_utilization_ >= target_utilization;
            };

        try {
            // 阵列模式：直接走阵列排样逻辑（不做直径搜索）
            if (params_.use_array_mode) {
                return optimize_array_mode(target_utilization, requestQuit, progress_callback);
            }

            double current_diameter = safe_to_double(layout_.sheets[0].diameter);

            // 如果启用Hodograph方法，先用它生成快速初始解
            if (params_.use_hodograph_initial) {
                if (generate_hodograph_initial_solution(current_diameter, requestQuit)) {
                    // 计算初始利用率
                    double initial_util = calculate_utilization();
                    if (initial_util > best_utilization_) {
                        best_utilization_ = initial_util;
                        layout_.best_utilization = best_utilization_;
                        layout_.best_result = layout_.sheet_parts;
                    }
                    if (progress_callback) {
                        progress_callback(layout_);
                    }
                    if (params_.stop_after_first_stage) {
                        return finish_after_first_stage();
                    }
                    if (best_utilization_ >= target_utilization) {
                        maybe_cgal_validate_and_checkpoint_best(requestQuit);
                        return true;
                    }
                }
            }

            // 首先尝试在当前直径下放置（如果Hodograph未启用或失败）
            if (!params_.use_hodograph_initial || best_utilization_ == 0.0) {
                // 尝试放置零件，即使只放置了部分零件也继续（不直接返回false）
                bool placed_any = try_place_all_parts(current_diameter, requestQuit, &progress_callback);
                if (!placed_any) {
                    // 如果连第一个零件都无法放置，才返回false
                    return false;
                }
                // 即使只放置了部分零件，也继续后续优化（紧凑化、直径优化等）
                double util_now = 0.0;
                try {
                    util_now = calculate_utilization();
                }
                catch (...) {
                    util_now = 0.0;
                }
                if (util_now > best_utilization_) {
                    best_utilization_ = util_now;
                    layout_.best_utilization = best_utilization_;
                    layout_.best_result = layout_.sheet_parts;
                }
                if (params_.stop_after_first_stage) {
                    return finish_after_first_stage();
                }
                if (best_utilization_ >= target_utilization) {
                    maybe_cgal_validate_and_checkpoint_best(requestQuit);
                    if (progress_callback) {
                        progress_callback(layout_);
                    }
                    return true;
                }
            }

            maybe_cgal_validate_and_checkpoint_best(requestQuit);

            // 如果用户已经请求退出，跳过后续紧凑化和直径优化，直接用当前结果
            if (should_stop(requestQuit)) {
                double util_now = calculate_utilization();
                best_utilization_ = util_now;
                layout_.best_utilization = best_utilization_;
                layout_.best_result = layout_.sheet_parts;
                maybe_cgal_validate_and_checkpoint_best(requestQuit);
                if (progress_callback) {
                    progress_callback(layout_);
                }
                return false;
            }

            // 检查上界/下界差距（如果启用）
            if (params_.use_bound_evaluation) {
                double bound_gap = calculate_bound_gap();
                // 如果差距已经足够小，可以提前停止优化
                if (bound_gap <= params_.bound_gap_threshold) {
                    // 差距已经很小，说明已经接近最优，可以提前停止
                    double current_util = calculate_utilization();
                    if (current_util > best_utilization_) {
                        best_utilization_ = current_util;
                        layout_.best_utilization = best_utilization_;
                        layout_.best_result = layout_.sheet_parts;
                    }
                    if (progress_callback) {
                        progress_callback(layout_);
                    }
                    return best_utilization_ >= target_utilization;
                }
            }

            // 紧凑化
            compact_layout(params_.compact_iterations, requestQuit, &progress_callback);

            local_flip_micro_shift_repair(requestQuit, &progress_callback);

            // 确保 best_result 是最新的最佳状态（即使被停止）
            double current_util = calculate_utilization();
            if (current_util > best_utilization_) {
                best_utilization_ = current_util;
                layout_.best_utilization = best_utilization_;
                layout_.best_result = layout_.sheet_parts;
                maybe_cgal_validate_and_checkpoint_best(requestQuit);
            }
            else {
                const double util_eps = 1e-9;
                if (current_util + util_eps >= layout_.best_utilization) {
                    layout_.best_result = layout_.sheet_parts;
                }
            }

            // 再次检查上界/下界差距
            if (params_.use_bound_evaluation) {
                double bound_gap = calculate_bound_gap();
                if (bound_gap <= params_.bound_gap_threshold) {
                    // 差距已经足够小，提前停止
                    if (progress_callback) {
                        progress_callback(layout_);
                    }
                    return best_utilization_ >= target_utilization;
                }
            }

            if (progress_callback) {
                progress_callback(layout_);
            }

            // 如果已达到目标，直接返回
            if (best_utilization_ >= target_utilization) {
                maybe_cgal_validate_and_checkpoint_best(requestQuit);
                if (params_.geometry_library == Parameters::GeometryLibrary::Clipper &&
                    !layout_.best_result.empty() && !layout_.best_result[0].empty()) {
                    bool final_ok = cgal_validate_no_overlap_for_parts(layout_.best_result[0], requestQuit);
                    if (!final_ok) {
                        rollback_to_last_validated_best();
                    }
                }
                return true;
            }

            // 优化直径
            bool diameter_success = optimize_diameter(target_utilization, requestQuit, progress_callback);

            // 停止时，确保 best_result 是最新的（可能在直径优化过程中有改进）
            if (should_stop(requestQuit)) {
                double final_util = calculate_utilization();
                if (final_util > best_utilization_) {
                    best_utilization_ = final_util;
                    layout_.best_utilization = best_utilization_;
                    layout_.best_result = layout_.sheet_parts;
                    maybe_cgal_validate_and_checkpoint_best(requestQuit);
                }
            }

            maybe_cgal_validate_and_checkpoint_best(requestQuit);
            if (params_.geometry_library == Parameters::GeometryLibrary::Clipper &&
                !layout_.best_result.empty() && !layout_.best_result[0].empty()) {
                bool final_ok = cgal_validate_no_overlap_for_parts(layout_.best_result[0], requestQuit);
                if (!final_ok) {
                    rollback_to_last_validated_best();
                    diameter_success = best_utilization_ >= target_utilization;
                }
            }

            return diameter_success;
        }
        catch (...) {
            return false;
        }
    }

    double CircleNesting::get_current_diameter() const {
        return current_diameter_;
    }

    double CircleNesting::calculate_shape_complexity(size_t shape_idx) const {
        if (shape_idx >= layout_.sheet_parts[0].size()) {
            return 0.0;
        }

        try {
            const auto& shape = layout_.sheet_parts[0][shape_idx];
            const auto& poly = *shape.base;

            // 计算面积
            double area = safe_to_double(geo::pwh_area(poly));
            if (area <= 0) return 0.0;

            // 计算周长
            double perimeter = 0.0;
            for (auto e = poly.outer_boundary().edges_begin();
                e != poly.outer_boundary().edges_end(); ++e) {
                double dx = safe_to_double(e->target().x() - e->source().x());
                double dy = safe_to_double(e->target().y() - e->source().y());
                perimeter += std::sqrt(dx * dx + dy * dy);
            }

            // 计算长宽比
            auto bbox = poly.bbox();
            double width = safe_to_double(bbox.xmax() - bbox.xmin());
            double height = safe_to_double(bbox.ymax() - bbox.ymin());
            double aspect_ratio = (width > height) ? width / height : height / width;

            // 计算顶点数（复杂度指标）
            size_t vertex_count = poly.outer_boundary().size();

            // 综合评分：面积大、周长/面积比大（复杂形状）、长宽比大（细长形状）优先
            // 这些形状通常更难放置，应该优先处理
            double complexity = area * (1.0 + perimeter / area * 0.1) * (1.0 + aspect_ratio * 0.1) * (1.0 + vertex_count * 0.01);

            return complexity;
        }
        catch (...) {
            return 0.0;
        }
    }

    std::vector<size_t> CircleNesting::smart_order_parts() const {
        std::vector<size_t> indices(layout_.poly_num);
        std::iota(indices.begin(), indices.end(), 0);

        // 按综合评分排序：复杂度高的优先
        std::sort(indices.begin(), indices.end(), [&](size_t i, size_t j) {
            try {
                double complexity_i = calculate_shape_complexity(i);
                double complexity_j = calculate_shape_complexity(j);
                return complexity_i > complexity_j;
            }
            catch (...) {
                return false;
            }
            });

        return indices;
    }

    // ============================================================================
    // 调度算法：尝试多种放置顺序，返回能放置最多零件且利用率最高的顺序
    // ============================================================================
    std::vector<size_t> CircleNesting::schedule_best_order(double diameter, volatile bool* requestQuit,
        std::function<void(const Layout&)>* progress_callback) {
        if (!params_.use_scheduling || params_.scheduling_attempts <= 1) {
            return smart_order_parts();
        }

        std::random_device rd;
        std::mt19937 gen(rd());

        auto better = [&](size_t placed_a, double util_a, size_t placed_b, double util_b) {
            if (placed_a != placed_b) {
                return placed_a > placed_b;
            }
            return util_a > util_b;
            };

        std::vector<size_t> best_order = smart_order_parts();
        size_t best_placed_count = 0;
        double best_util = 0.0;

        // 内存优化：只在找到更好结果时才保存布局
        bool has_best_layout = false;
        std::vector<std::vector<TransformedShape>> best_layout;

        // 内存优化：使用 move 语义备份，减少拷贝
        auto original_parts = std::move(layout_.sheet_parts);
        layout_.sheet_parts = original_parts; // 恢复一份用于操作

        // 内存优化：预分配 candidate_order
        std::vector<size_t> candidate_order;
        candidate_order.reserve(layout_.poly_num);

        // 内存优化：预分配 placed_indices
        std::vector<size_t> placed_indices;
        placed_indices.reserve(layout_.poly_num);

        // 内存优化：复用空列表
        const std::vector<size_t> empty_placed;

        auto shape_area = [&](size_t idx) -> double {
            try {
                return safe_to_double(geo::pwh_area(*original_parts[0][idx].base));
            }
            catch (...) {
                return 0.0;
            }
            };

        struct GroupInfo {
            size_t key;
            std::vector<size_t> indices;
            double area_sum;
        };

        std::vector<GroupInfo> groups;
        groups.reserve(layout_.poly_num);
        {
            std::unordered_map<size_t, size_t> group_pos;
            group_pos.reserve(layout_.poly_num);
            for (size_t i = 0; i < layout_.poly_num; ++i) {
                size_t key = static_cast<size_t>(original_parts[0][i].item_idx);
                auto it = group_pos.find(key);
                if (it == group_pos.end()) {
                    GroupInfo g;
                    g.key = key;
                    g.indices.clear();
                    g.indices.push_back(i);
                    g.area_sum = shape_area(i);
                    groups.push_back(std::move(g));
                    group_pos[key] = groups.size() - 1;
                }
                else {
                    auto& g = groups[it->second];
                    g.indices.push_back(i);
                    g.area_sum += shape_area(i);
                }
            }
        }

        std::vector<GroupInfo> groups_by_area = groups;
        std::sort(groups_by_area.begin(), groups_by_area.end(), [&](const GroupInfo& a, const GroupInfo& b) {
            if (a.area_sum != b.area_sum) {
                return a.area_sum > b.area_sum;
            }
            return a.indices.size() > b.indices.size();
            });

        auto build_group_order = [&](bool shuffle_groups, bool shuffle_within, bool sort_within_by_area) {
            std::vector<GroupInfo> gs = groups_by_area;
            if (shuffle_groups) {
                std::shuffle(gs.begin(), gs.end(), gen);
            }
            candidate_order.clear();
            candidate_order.reserve(layout_.poly_num);
            for (auto& g : gs) {
                if (sort_within_by_area) {
                    std::sort(g.indices.begin(), g.indices.end(), [&](size_t i, size_t j) {
                        return shape_area(i) > shape_area(j);
                        });
                }
                if (shuffle_within) {
                    std::shuffle(g.indices.begin(), g.indices.end(), gen);
                }
                candidate_order.insert(candidate_order.end(), g.indices.begin(), g.indices.end());
            }
            };

        auto eval_order = [&](const std::vector<size_t>& order, std::vector<std::vector<TransformedShape>>* out_layout,
            size_t& out_placed, double& out_util) {
                layout_.sheet_parts = original_parts;
                update_circle_diameter(diameter);

                placed_indices.clear();
                double radius = diameter / 2.0;
                double outside_offset = -radius * 2.0;
                for (auto& s : layout_.sheet_parts[0]) {
                    s.set_translate(geo::FT(outside_offset), geo::FT(outside_offset));
                    s.update();
                }

                if (!order.empty()) {
                    size_t first_idx = order[0];
                    if (!can_never_fit_in_circle(first_idx)) {
                        auto& shape = layout_.sheet_parts[0][first_idx];
                        shape.update();
                        auto bbox = shape.transformed.bbox();
                        double shape_width = safe_to_double(bbox.xmax() - bbox.xmin());
                        double shape_height = safe_to_double(bbox.ymax() - bbox.ymin());
                        shape.set_translate(geo::FT(radius - shape_width / 2.0),
                            geo::FT(radius - shape_height / 2.0));
                        shape.update();

                        if (is_valid_placement(first_idx, empty_placed)) {
                            placed_indices.push_back(first_idx);
                        }
                        else if (place_shape_with_nfp(first_idx, empty_placed, params_.try_all_rotations, requestQuit)) {
                            placed_indices.push_back(first_idx);
                        }
                    }
                }

                for (size_t i = 1; i < order.size(); ++i) {
                    if (should_stop(requestQuit)) {
                        break;
                    }
                    size_t shape_idx = order[i];
                    if (can_never_fit_in_circle(shape_idx)) {
                        continue;
                    }
                    bool placed = place_shape_with_nfp(shape_idx, placed_indices, params_.try_all_rotations, requestQuit);
                    if (placed) {
                        placed_indices.push_back(shape_idx);
                        if (progress_callback && (placed_indices.size() % 5 == 0)) {
                            layout_.best_result = layout_.sheet_parts;
                            layout_.best_utilization = best_utilization_;
                            (*progress_callback)(layout_);
                        }
                    }
                }

                out_placed = placed_indices.size();
                out_util = 0.0;
                try {
                    out_util = calculate_utilization();
                }
                catch (...) {
                    out_util = 0.0;
                }

                if (out_layout) {
                    *out_layout = layout_.sheet_parts;
                }
            };

        auto ils_improve = [&](std::vector<size_t>& order, size_t& best_placed, double& best_u,
            std::vector<std::vector<TransformedShape>>& best_l) {
                if (order.size() < 6) {
                    return;
                }
                std::uniform_int_distribution<int> move_type(0, 2);
                const size_t max_iters = std::min<size_t>(80, std::max<size_t>(10, layout_.poly_num / 20));
                for (size_t it = 0; it < max_iters; ++it) {
                    if (should_stop(requestQuit)) {
                        break;
                    }

                    std::vector<size_t> cand = order;
                    size_t n0 = cand.size();
                    std::uniform_int_distribution<size_t> pick(0, n0 - 1);
                    size_t a = pick(gen);
                    size_t b = pick(gen);
                    if (a == b) {
                        continue;
                    }
                    int mt = move_type(gen);
                    if (mt == 0) {
                        std::swap(cand[a], cand[b]);
                    }
                    else if (mt == 1) {
                        size_t from = a;
                        size_t to = b;
                        size_t v = cand[from];
                        cand.erase(cand.begin() + static_cast<std::ptrdiff_t>(from));
                        if (to > from) {
                            --to;
                        }
                        cand.insert(cand.begin() + static_cast<std::ptrdiff_t>(to), v);
                    }
                    else {
                        size_t l = std::min(a, b);
                        size_t r = std::max(a, b);
                        if (r > l + 1) {
                            std::reverse(cand.begin() + static_cast<std::ptrdiff_t>(l),
                                cand.begin() + static_cast<std::ptrdiff_t>(r + 1));
                        }
                    }

                    size_t placed_now = 0;
                    double util_now = 0.0;
                    std::vector<std::vector<TransformedShape>> layout_now;
                    eval_order(cand, &layout_now, placed_now, util_now);
                    if (better(placed_now, util_now, best_placed, best_u)) {
                        order = std::move(cand);
                        best_placed = placed_now;
                        best_u = util_now;
                        best_l = std::move(layout_now);
                    }
                }
            };

        for (size_t attempt = 0; attempt < params_.scheduling_attempts; ++attempt) {
            if (should_stop(requestQuit)) break;

            candidate_order.clear();
            if (attempt == 0) {
                candidate_order = smart_order_parts();
            }
            else if (attempt == 1) {
                candidate_order.resize(layout_.poly_num);
                std::iota(candidate_order.begin(), candidate_order.end(), 0);
                std::sort(candidate_order.begin(), candidate_order.end(), [&](size_t i, size_t j) {
                    return shape_area(i) > shape_area(j);
                    });
            }
            else if (attempt == 2) {
                build_group_order(false, false, true);
            }
            else if (attempt == 3) {
                build_group_order(true, false, true);
            }
            else if (attempt == 4) {
                build_group_order(true, true, false);
            }
            else if (params_.use_random_shuffle) {
                if (attempt % 2 == 0) {
                    build_group_order(true, true, true);
                }
                else {
                    candidate_order = smart_order_parts();
                    std::shuffle(candidate_order.begin(), candidate_order.end(), gen);
                }
            }
            else {
                continue;
            }

            size_t placed_now = 0;
            double util_now = 0.0;
            std::vector<std::vector<TransformedShape>> layout_now;
            eval_order(candidate_order, &layout_now, placed_now, util_now);

            ils_improve(candidate_order, placed_now, util_now, layout_now);

            if (better(placed_now, util_now, best_placed_count, best_util)) {
                best_placed_count = placed_now;
                best_util = util_now;
                best_order = candidate_order;
                best_layout = layout_now;
                has_best_layout = true;
            }
        }

        // 恢复最佳布局
        if (has_best_layout) {
            layout_.sheet_parts = std::move(best_layout);
        }
        else {
            layout_.sheet_parts = std::move(original_parts);
        }

        return best_order;
    }

    double CircleNesting::binary_search_diameter(double min_diameter, double max_diameter,
        double target_utilization, volatile bool* requestQuit,
        std::function<void(const Layout&)> progress_callback) {
        double best_diameter = max_diameter;
        size_t best_placed_count = 0;
        double best_utilization = 0.0;
        const double util_eps = 1e-9;
        const double tolerance = 0.001; // 直径容差
        const size_t max_iterations = 50;

        for (size_t iter = 0; iter < max_iterations; ++iter) {
            if (should_stop(requestQuit)) {
                break;
            }

            if (max_diameter - min_diameter < tolerance) {
                break;
            }

            double test_diameter = (min_diameter + max_diameter) / 2.0;

            // 使用调度算法尝试多种顺序，找到最佳放置方案
            schedule_best_order(test_diameter, requestQuit, &progress_callback);

            compact_layout(params_.compact_iterations / 2, requestQuit, &progress_callback);
            qWarning() << "[LOCAL_REPAIR] call: test_diameter=" << test_diameter;
            local_flip_micro_shift_repair(requestQuit, &progress_callback);
            qWarning() << "[LOCAL_REPAIR] return: test_diameter=" << test_diameter;

            double util = 0.0;
            try {
                util = calculate_utilization();
            }
            catch (...) {
                util = 0.0;
            }

            size_t placed_count = 0;
            {
                const double radius = test_diameter / 2.0;
                const double center_x = radius;
                const double center_y = radius;
                const double radius_sq = radius * radius;
                const double tol = 1e-10;
                for (const auto& shape : layout_.sheet_parts[0]) {
                    auto txy = shape.get_translate_double();
                    if (txy.first < 0.0 && txy.second < 0.0) {
                        continue;
                    }
                    bool inside = true;
                    const auto& ob = shape.transformed.outer_boundary();
                    for (auto vit = ob.vertices_begin(); vit != ob.vertices_end(); ++vit) {
                        double x = safe_to_double(vit->x());
                        double y = safe_to_double(vit->y());
                        double dx = x - center_x;
                        double dy = y - center_y;
                        double d2 = dx * dx + dy * dy;
                        if (d2 > radius_sq * (1.0 + tol)) {
                            inside = false;
                            break;
                        }
                    }
                    if (inside) {
                        ++placed_count;
                    }
                }
            }

            // 记录最优：放入数量优先，其次利用率，其次更小直径
            if (placed_count > best_placed_count ||
                (placed_count == best_placed_count && (util > best_utilization + util_eps)) ||
                (placed_count == best_placed_count && std::abs(util - best_utilization) <= util_eps &&
                    test_diameter + tolerance < best_diameter)) {
                best_placed_count = placed_count;
                best_utilization = util;
                best_diameter = test_diameter;

                layout_.best_result = layout_.sheet_parts;
                layout_.best_utilization = util;

                if (progress_callback) {
                    progress_callback(layout_);
                }
            }

            // 二分方向：
            // - 放不全：必须增大直径，才可能放入更多
            // - 放全：尝试缩小直径以提高紧凑度/利用率
            if (placed_count < layout_.poly_num) {
                min_diameter = test_diameter;
            }
            else {
                max_diameter = test_diameter;
            }
        }

        return best_diameter;
    }

    // ============================================================================
    // 上界/下界评估：计算改进的下界（考虑形状约束和旋转限制）
    // ============================================================================
    double CircleNesting::calculate_improved_lower_bound() const {
        if (layout_.sheet_parts.empty() || layout_.sheet_parts[0].empty()) {
            return 0.0;
        }

        try {
            double total_area = safe_to_double(layout_.area);
            if (total_area <= 0) {
                return 0.0;
            }

            // 方法1：基础面积下界
            double basic_lower_bound = std::sqrt(4.0 * total_area / Sheet::kPi);

            // 方法2：考虑形状复杂度的下界
            // 计算所有零件的平均长宽比和复杂度
            double total_complexity = 0.0;
            double max_bbox_diagonal = 0.0;
            size_t valid_shapes = 0;

            for (size_t i = 0; i < layout_.poly_num; ++i) {
                try {
                    const auto& shape = layout_.sheet_parts[0][i];
                    const auto& poly = *shape.base;

                    double area = safe_to_double(geo::pwh_area(poly));
                    if (area <= 0) continue;

                    auto bbox = poly.bbox();
                    double width = safe_to_double(bbox.xmax() - bbox.xmin());
                    double height = safe_to_double(bbox.ymax() - bbox.ymin());
                    double diagonal = std::sqrt(width * width + height * height);
                    max_bbox_diagonal = std::max(max_bbox_diagonal, diagonal);

                    // 计算周长
                    double perimeter = 0.0;
                    for (auto e = poly.outer_boundary().edges_begin();
                        e != poly.outer_boundary().edges_end(); ++e) {
                        double dx = safe_to_double(e->target().x() - e->source().x());
                        double dy = safe_to_double(e->target().y() - e->source().y());
                        perimeter += std::sqrt(dx * dx + dy * dy);
                    }

                    // 复杂度 = 面积 * (1 + 周长/面积比) * (1 + 长宽比)
                    double aspect_ratio = (width > height) ? width / height : height / width;
                    double complexity = area * (1.0 + perimeter / area * 0.1) * (1.0 + aspect_ratio * 0.1);
                    total_complexity += complexity;
                    valid_shapes++;
                }
                catch (...) {
                    continue;
                }
            }

            if (valid_shapes == 0) {
                return basic_lower_bound;
            }

            // 方法3：考虑旋转约束的下界
            // 如果允许旋转，下界可能更小；如果不允许，下界可能更大
            double rotation_factor = 1.0;
            if (params_.try_all_rotations) {
                // 允许旋转时，可以更紧凑，下界可以更小（乘以0.95-0.98）
                rotation_factor = 0.96;
            }
            else {
                // 不允许旋转时，可能需要更多空间（乘以1.02-1.05）
                rotation_factor = 1.03;
            }

            // 方法4：考虑最大零件尺寸的下界
            // 至少需要能放下最大的零件
            double max_part_lower_bound = max_bbox_diagonal * 1.1; // 留10%余量

            // 综合下界：取最大值，确保所有约束都满足
            double improved_lower_bound = std::max(
                basic_lower_bound * rotation_factor,
                max_part_lower_bound
            );

            // 考虑平均复杂度的影响（复杂形状需要更多空间）
            double avg_complexity = total_complexity / valid_shapes;
            double avg_area = total_area / valid_shapes;
            double complexity_ratio = avg_complexity / avg_area;
            if (complexity_ratio > 1.5) {
                // 如果平均复杂度较高，下界需要增加
                improved_lower_bound *= (1.0 + (complexity_ratio - 1.5) * 0.1);
            }

            return improved_lower_bound;
        }
        catch (...) {
            // 如果计算失败，返回基础下界
            double total_area = safe_to_double(layout_.area);
            return std::sqrt(4.0 * total_area / Sheet::kPi);
        }
    }

    // ============================================================================
    // 上界/下界评估：计算上界/下界差距（返回差距百分比，0-1之间）
    // ============================================================================
    double CircleNesting::calculate_bound_gap() const {
        if (!params_.use_bound_evaluation) {
            return 1.0; // 如果未启用，返回100%差距（表示未知）
        }

        try {
            // 计算当前上界（实际利用率）
            double upper_bound_util = calculate_utilization();
            if (upper_bound_util <= 0) {
                return 1.0;
            }

            // 计算下界（理论最优利用率）
            double current_diameter = safe_to_double(layout_.sheets[0].diameter);
            double lower_bound_diameter;

            if (params_.use_improved_lower_bound) {
                lower_bound_diameter = calculate_improved_lower_bound();
            }
            else {
                lower_bound_diameter = calculate_theoretical_min_diameter();
            }

            if (lower_bound_diameter <= 0 || current_diameter <= 0) {
                return 1.0;
            }

            // 重要：这里不能用 layout_.area（所有零件总面积）来算下界利用率。
            // 因为当前布局可能只放下了部分零件，用总面积会导致 lower_bound_util 很容易 > 1 被夹到 1，
            // 进而 gap 被夹到 0，触发“已经接近最优”的误判，导致在 0.6 左右提前停止。
            // 这里改为基于“当前已放置面积”估计下界：placed_area = upper_util * current_circle_area。
            double current_circle_area = Sheet::kPi * (current_diameter / 2.0) * (current_diameter / 2.0);
            double placed_area = upper_bound_util * current_circle_area;
            double lower_bound_circle_area = Sheet::kPi * (lower_bound_diameter / 2.0) * (lower_bound_diameter / 2.0);
            double lower_bound_util = (lower_bound_circle_area > 0) ? (placed_area / lower_bound_circle_area) : 0.0;

            // 限制在合理范围内
            if (lower_bound_util > 1.0) lower_bound_util = 1.0;
            if (lower_bound_util < 0.0) lower_bound_util = 0.0;

            // 差距 = (上界利用率 - 下界利用率) / 下界利用率
            // 或者更直观：差距 = (当前直径 - 下界直径) / 下界直径
            double diameter_gap = (current_diameter - lower_bound_diameter) / lower_bound_diameter;
            double util_gap = (upper_bound_util - lower_bound_util) / std::max(lower_bound_util, 0.01);

            // 返回较小的差距（更保守的估计），并确保非负
            double gap = std::min(diameter_gap, util_gap);
            if (gap < 0) gap = 0;

            // 限制在0-1之间
            if (gap > 1) gap = 1;

            return gap;
        }
        catch (...) {
            return 1.0;
        }
    }

    // ============================================================================
    // Hodograph方法：生成快速初始解
    // Hodograph（速度图）是一种贪心策略，通过分析零件的"速度"（放置难度）
    // 来决定放置顺序，快速生成初始布局
    // ============================================================================
    bool CircleNesting::generate_hodograph_initial_solution(double diameter, volatile bool* requestQuit) {
        if (layout_.sheet_parts.empty() || layout_.sheet_parts[0].empty()) {
            return false;
        }

        update_circle_diameter(diameter);

        try {
            // 重置所有零件位置
            double radius = diameter / 2.0;
            double outside_offset = -radius * 2.0;
            for (auto& shape : layout_.sheet_parts[0]) {
                shape.set_translate(geo::FT(outside_offset), geo::FT(outside_offset));
                shape.update();
            }

            // Hodograph方法：计算每个零件的"速度"（放置难度）
            // 速度 = 面积 / (周长 * 长宽比)，值越大越容易放置
            struct ShapeInfo {
                size_t idx;
                double velocity;  // 放置速度（越大越容易）
                double area;
                double perimeter;
                double aspect_ratio;
            };

            std::vector<ShapeInfo> shape_infos;
            shape_infos.reserve(layout_.poly_num);

            for (size_t i = 0; i < layout_.poly_num; ++i) {
                if (should_stop(requestQuit)) {
                    return false;
                }

                try {
                    const auto& shape = layout_.sheet_parts[0][i];
                    const auto& poly = *shape.base;

                    double area = safe_to_double(geo::pwh_area(poly));
                    if (area <= 0) continue;

                    // 计算周长
                    double perimeter = 0.0;
                    for (auto e = poly.outer_boundary().edges_begin();
                        e != poly.outer_boundary().edges_end(); ++e) {
                        double dx = safe_to_double(e->target().x() - e->source().x());
                        double dy = safe_to_double(e->target().y() - e->source().y());
                        perimeter += std::sqrt(dx * dx + dy * dy);
                    }

                    // 计算长宽比
                    auto bbox = poly.bbox();
                    double width = safe_to_double(bbox.xmax() - bbox.xmin());
                    double height = safe_to_double(bbox.ymax() - bbox.ymin());
                    double aspect_ratio = (width > height) ? width / height : height / width;

                    // Hodograph速度 = 面积 / (周长 * 长宽比)
                    // 面积大、周长小、长宽比接近1的零件更容易放置
                    double velocity = (perimeter > 0 && aspect_ratio > 0)
                        ? area / (perimeter * aspect_ratio)
                        : area;

                    shape_infos.push_back({ i, velocity, area, perimeter, aspect_ratio });
                }
                catch (...) {
                    continue;
                }
            }

            if (shape_infos.empty()) {
                return false;
            }

            // 按速度从大到小排序（容易放置的优先）
            std::sort(shape_infos.begin(), shape_infos.end(),
                [](const ShapeInfo& a, const ShapeInfo& b) {
                    return a.velocity > b.velocity;
                });

            // 放置零件：使用简单的径向放置策略
            std::vector<size_t> placed_indices;
            double center_x = radius;
            double center_y = radius;
            double angle_step = 2.0 * Sheet::kPi / shape_infos.size();
            double radius_step = radius * 0.3 / shape_infos.size();

            for (size_t i = 0; i < shape_infos.size(); ++i) {
                if (should_stop(requestQuit)) {
                    return !placed_indices.empty();
                }

                size_t shape_idx = shape_infos[i].idx;
                auto& shape = layout_.sheet_parts[0][shape_idx];
                shape.update();

                // 计算放置位置：螺旋式放置
                double angle = angle_step * i;
                double r = radius_step * i + radius * 0.2; // 从中心20%半径开始
                double x = center_x + r * std::cos(angle);
                double y = center_y + r * std::sin(angle);

                auto bbox = shape.transformed.bbox();
                double shape_width = safe_to_double(bbox.xmax() - bbox.xmin());
                double shape_height = safe_to_double(bbox.ymax() - bbox.ymin());

                // 将零件中心放在计算位置
                shape.set_translate(geo::FT(x - shape_width / 2.0),
                    geo::FT(y - shape_height / 2.0));
                shape.update();

                // 检查是否有效
                if (is_valid_placement(shape_idx, placed_indices)) {
                    placed_indices.push_back(shape_idx);
                }
                else {
                    // 如果简单位置无效，尝试使用NFP方法
                    if (place_shape_with_nfp(shape_idx, placed_indices, false)) {
                        placed_indices.push_back(shape_idx);
                    }
                    else {
                        // 如果还是失败，移到板外
                        shape.set_translate(geo::FT(outside_offset), geo::FT(outside_offset));
                        shape.update();
                    }
                }
            }

            // 将未放置的零件移到板外
            std::set<size_t> placed_set(placed_indices.begin(), placed_indices.end());
            for (size_t i = 0; i < layout_.poly_num; ++i) {
                if (placed_set.find(i) == placed_set.end()) {
                    auto& shape = layout_.sheet_parts[0][i];
                    double offset_x = outside_offset + i * radius * 0.1;
                    double offset_y = outside_offset;
                    shape.set_translate(geo::FT(offset_x), geo::FT(offset_y));
                    shape.update();
                }
            }

            return !placed_indices.empty();
        }
        catch (...) {
            return false;
        }
    }

    // ============================================================================
    // 废料最小化：计算当前布局的废料区域（圆 - 所有已放置零件的并集）
    // ============================================================================
    std::vector<geo::Polygon_with_holes_2> CircleNesting::calculate_waste_regions(
        const std::vector<size_t>& placed_indices) const {
        std::vector<geo::Polygon_with_holes_2> waste_regions;

        if (placed_indices.empty()) {
            // 如果没有已放置零件，整个圆都是"废料"（实际上是可以放置的区域）
            waste_regions.push_back(layout_.sheets[0].sheet);
            return waste_regions;
        }

        try {
            // 获取圆形板材
            geo::Polygon_with_holes_2 sheet_pwh = layout_.sheets[0].sheet;

            if (params_.geometry_library == Parameters::GeometryLibrary::Clipper) {
                // 使用 Clipper
                // 计算所有已放置零件的并集
                geo::Polygon_with_holes_2 placed_union;
                bool first = true;

                for (size_t idx : placed_indices) {
                    if (idx >= layout_.sheet_parts[0].size()) {
                        continue;
                    }
                    try {
                        const auto& shape = layout_.sheet_parts[0][idx];
                        // 只计算在圆内的部分
                        auto inters_result = geo::GeometryOperations::intersection(shape.transformed, sheet_pwh);

                        if (inters_result.is_empty || inters_result.polygons.empty()) {
                            continue;
                        }

                        // 合并到并集
                        for (const auto& inter_poly : inters_result.polygons) {
                            if (first) {
                                placed_union = inter_poly;
                                first = false;
                            }
                            else {
                                auto join_result = geo::GeometryOperations::join(placed_union, inter_poly);
                                if (!join_result.is_empty && !join_result.polygons.empty()) {
                                    placed_union = join_result.polygons[0];
                                }
                            }
                        }
                    }
                    catch (...) {
                        // 忽略单个零件的错误
                    }
                }

                // 计算废料区域 = 圆 - 已放置零件的并集
                if (!first) {
                    auto diff_result = geo::GeometryOperations::difference(sheet_pwh, placed_union);
                    waste_regions = diff_result.polygons;
                }
                else {
                    // 如果没有已放置零件，整个圆都是废料区域
                    waste_regions.push_back(sheet_pwh);
                }
            }
            else {
                // 使用 CGAL（默认）
                // 计算所有已放置零件的并集
                geo::Polygon_set_2 placed_union(geo::traits);

                for (size_t idx : placed_indices) {
                    if (idx >= layout_.sheet_parts[0].size()) {
                        continue;
                    }
                    try {
                        const auto& shape = layout_.sheet_parts[0][idx];
                        // 只计算在圆内的部分
                        std::vector<geo::Polygon_with_holes_2> inters;
                        try {
                            CGAL::intersection(shape.transformed, sheet_pwh, std::back_inserter(inters));
                        }
                        catch (...) {
                            inters.clear();
                        }

                        for (const auto& p : inters) {
                            try {
                                placed_union.join(p.outer_boundary());
                            }
                            catch (...) {
                                // 忽略错误
                            }
                        }
                    }
                    catch (...) {
                        // 忽略单个零件的错误
                    }
                }

                // 计算废料区域 = 圆 - 已放置零件的并集
                geo::Polygon_set_2 waste_set(geo::traits);
                waste_set.insert(sheet_pwh.outer_boundary());
                waste_set.difference(placed_union);

                // 提取废料多边形
                waste_set.polygons_with_holes(std::back_inserter(waste_regions));
            }

            // 过滤：只保留面积足够大的废料区域
            double min_waste_area = safe_to_double(geo::pwh_area(sheet_pwh)) * params_.min_waste_area_ratio;
            waste_regions.erase(
                std::remove_if(waste_regions.begin(), waste_regions.end(),
                    [&](const geo::Polygon_with_holes_2& pwh) {
                        try {
                            double area = safe_to_double(geo::pwh_area(pwh));
                            return area < min_waste_area;
                        }
                        catch (...) {
                            return true; // 如果计算失败，移除
                        }
                    }),
                waste_regions.end()
            );

            // 按面积从大到小排序（优先使用大废料区域）
            std::sort(waste_regions.begin(), waste_regions.end(),
                [&](const geo::Polygon_with_holes_2& a, const geo::Polygon_with_holes_2& b) {
                    try {
                        double area_a = safe_to_double(geo::pwh_area(a));
                        double area_b = safe_to_double(geo::pwh_area(b));
                        return area_a > area_b;
                    }
                    catch (...) {
                        return false;
                    }
                });
        }
        catch (...) {
            // 如果计算失败，返回空列表
            waste_regions.clear();
        }

        return waste_regions;
    }

    // ============================================================================
    // 废料最小化：在废料区域生成候选点（优先在最大废料区域）
    // ============================================================================
    std::vector<geo::Point_2> CircleNesting::generate_waste_candidates(
        size_t shape_idx,
        const std::vector<size_t>& placed_indices,
        const std::vector<geo::Polygon_with_holes_2>& waste_regions) const {
        std::vector<geo::Point_2> candidates;

        if (shape_idx >= layout_.sheet_parts[0].size() || waste_regions.empty()) {
            return candidates;
        }

        try {
            const auto& shape = layout_.sheet_parts[0][shape_idx];
            auto bbox = shape.transformed.bbox();
            double shape_width = safe_to_double(bbox.xmax() - bbox.xmin());
            double shape_height = safe_to_double(bbox.ymax() - bbox.ymin());
            double shape_diagonal = std::sqrt(shape_width * shape_width + shape_height * shape_height);

            // 遍历废料区域（已按面积从大到小排序）
            for (const auto& waste_region : waste_regions) {
                if (candidates.size() >= params_.max_waste_candidates) {
                    break;
                }

                try {
                    // 方法1：在废料区域的顶点附近生成候选点
                    for (auto v = waste_region.outer_boundary().vertices_begin();
                        v != waste_region.outer_boundary().vertices_end(); ++v) {
                        if (candidates.size() >= params_.max_waste_candidates) {
                            break;
                        }

                        // 将零件中心放在顶点附近
                        double x = safe_to_double(v->x()) - shape_width / 2.0;
                        double y = safe_to_double(v->y()) - shape_height / 2.0;
                        candidates.push_back(geo::Point_2(geo::FT(x), geo::FT(y)));
                    }

                    // 方法2：在废料区域的边界上采样点
                    size_t samples_per_edge = 3;
                    for (auto e = waste_region.outer_boundary().edges_begin();
                        e != waste_region.outer_boundary().edges_end(); ++e) {
                        if (candidates.size() >= params_.max_waste_candidates) {
                            break;
                        }

                        auto source = e->source();
                        auto target = e->target();
                        for (size_t s = 1; s < samples_per_edge; ++s) {
                            double t = double(s) / samples_per_edge;
                            double x = safe_to_double(source.x() * (1.0 - t) + target.x() * t) - shape_width / 2.0;
                            double y = safe_to_double(source.y() * (1.0 - t) + target.y() * t) - shape_height / 2.0;
                            candidates.push_back(geo::Point_2(geo::FT(x), geo::FT(y)));
                        }
                    }

                    // 方法3：在废料区域的质心附近生成候选点
                    try {
                        double total_area = safe_to_double(geo::pwh_area(waste_region));
                        if (total_area > 0) {
                            double cx = 0.0, cy = 0.0;
                            double weighted_sum_x = 0.0, weighted_sum_y = 0.0;

                            // 计算外边界质心（简化：使用顶点平均）
                            size_t vertex_count = 0;
                            for (auto v = waste_region.outer_boundary().vertices_begin();
                                v != waste_region.outer_boundary().vertices_end(); ++v) {
                                cx += safe_to_double(v->x());
                                cy += safe_to_double(v->y());
                                vertex_count++;
                            }
                            if (vertex_count > 0) {
                                cx /= vertex_count;
                                cy /= vertex_count;

                                // 在质心附近生成几个候选点
                                for (int offset = -1; offset <= 1; offset += 2) {
                                    if (candidates.size() >= params_.max_waste_candidates) {
                                        break;
                                    }
                                    double x = cx - shape_width / 2.0 + offset * shape_width * 0.1;
                                    double y = cy - shape_height / 2.0 + offset * shape_height * 0.1;
                                    candidates.push_back(geo::Point_2(geo::FT(x), geo::FT(y)));
                                }
                            }
                        }
                    }
                    catch (...) {
                        // 如果质心计算失败，跳过
                    }
                }
                catch (...) {
                    // 如果处理某个废料区域失败，继续下一个
                    continue;
                }
            }
        }
        catch (...) {
            // 如果生成失败，返回已生成的候选点
        }

        return candidates;
    }

    // ============================================================================
    // 废料最小化：优先在废料区域放置零件
    // ============================================================================
    bool CircleNesting::place_shape_in_waste(size_t shape_idx,
        const std::vector<size_t>& placed_indices,
        bool try_all_rotations) {
        if (shape_idx >= layout_.sheet_parts[0].size()) {
            return false;
        }

        auto& shape = layout_.sheet_parts[0][shape_idx];

        try {
            // 计算废料区域
            auto waste_regions = calculate_waste_regions(placed_indices);
            if (waste_regions.empty()) {
                // 如果没有废料区域，回退到普通放置
                return place_shape_with_nfp(shape_idx, placed_indices, try_all_rotations, nullptr, /*allow_waste_minimization=*/false);
            }

            // 在废料区域生成候选点
            auto waste_candidates = generate_waste_candidates(shape_idx, placed_indices, waste_regions);
            if (waste_candidates.empty()) {
                // 如果无法生成废料候选点，回退到普通放置
                return place_shape_with_nfp(shape_idx, placed_indices, try_all_rotations, nullptr, /*allow_waste_minimization=*/false);
            }

            // 尝试每个旋转角度
            std::vector<uint32_t> rotations_to_try;
            if (try_all_rotations && params_.try_all_rotations) {
                size_t max_rot = std::min(static_cast<size_t>(shape.allowed_rotations),
                    static_cast<size_t>(params_.max_rotation_attempts));
                for (uint32_t rot = 0; rot < max_rot; ++rot) {
                    rotations_to_try.push_back(rot);
                }
            }
            else {
                rotations_to_try.push_back(shape.get_rotation());
            }

            uint32_t original_rotation = shape.get_rotation();
            double radius = safe_to_double(layout_.sheets[0].diameter) / 2.0;
            double center_x = radius;
            double center_y = radius;

            geo::Point_2 best_point;
            uint32_t best_rotation = shape.get_rotation();
            double best_score = std::numeric_limits<double>::max();
            bool found = false;

            for (uint32_t rot : rotations_to_try) {
                shape.set_rotation(rot);
                shape.update();

                // 尝试每个废料候选点
                for (const auto& candidate : waste_candidates) {
                    auto old_x = shape.get_translate_ft_x();
                    auto old_y = shape.get_translate_ft_y();

                    shape.set_translate(candidate.x(), candidate.y());
                    shape.update();

                    // 检查是否有效
                    if (is_valid_placement(shape_idx, placed_indices)) {
                        // 评分：优先选择在最大废料区域的位置
                        // 计算到圆心的距离（鼓励靠近中心）
                        double dx = safe_to_double(candidate.x()) - center_x;
                        double dy = safe_to_double(candidate.y()) - center_y;
                        double dist_sq = dx * dx + dy * dy;

                        // 优先选择靠近中心且在废料区域的位置
                        double score = dist_sq;

                        if (score < best_score) {
                            best_score = score;
                            best_point = candidate;
                            best_rotation = rot;
                            found = true;
                        }
                    }

                    // 恢复
                    shape.set_translate(old_x, old_y);
                    shape.update();
                }
            }

            if (found) {
                shape.set_rotation(best_rotation);
                shape.set_translate(best_point.x(), best_point.y());
                shape.update();
                return true;
            }
            else {
                // 恢复原始旋转
                shape.set_rotation(original_rotation);
                shape.update();
                // 如果废料区域放置失败，回退到普通放置
                return place_shape_with_nfp(shape_idx, placed_indices, try_all_rotations, nullptr, /*allow_waste_minimization=*/false);
            }
        }
        catch (...) {
            // 如果出错，回退到普通放置
            return place_shape_with_nfp(shape_idx, placed_indices, try_all_rotations, nullptr, /*allow_waste_minimization=*/false);
        }
    }

} // namespace nesting



