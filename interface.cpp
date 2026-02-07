#include "interface.h"
#include "circle_nesting.h"
#include <ctime>
#include <iostream>

namespace nesting {

    void start(const size_t max_time,
        Layout& layout,
        std::function<void(const Solution&)> ProgressHandler,
        volatile bool* requestQuit) {
        // 圆形板材专用：直接使用高利用率算法
        // 这里默认使用高质量模式（fast_mode = false）
        start_ga(max_time, layout, ProgressHandler, requestQuit, false);
    }

    void start_ga(const size_t max_time,
        Layout& layout,
        std::function<void(const Solution&)> ProgressHandler,
        volatile bool* requestQuit,
        bool fast_mode,
        bool use_clipper) {
        // 只支持圆形板材
        if (layout.sheets.empty() || layout.sheets[0].type != Sheet::ShapeType::Circle) {
            throw std::runtime_error("只支持圆形板材排样");
        }

        clock_t start_clock = clock();
        
        CircleNesting circle_nesting(layout);
        circle_nesting.set_time_limit_seconds(0);
        circle_nesting.set_quality_gate(0.0);

        const size_t hard_limit = 3 * 60 * 60;
        circle_nesting.set_hard_time_limit_seconds(hard_limit);

        // 设置参数
        CircleNesting::Parameters params;

        fast_mode = false;
        use_clipper = true;

        std::cerr << "[start_ga] fast_mode=" << (fast_mode ? 1 : 0)
                  << " use_clipper=" << (use_clipper ? 1 : 0) << std::endl;
        
        // 根据参数选择几何库（GUI 控制）
        if (use_clipper) {
            params.geometry_library = CircleNesting::Parameters::GeometryLibrary::Clipper;
        } else {
            params.geometry_library = CircleNesting::Parameters::GeometryLibrary::CGAL;
        }

        std::cerr << "[start_ga] geometry_library="
                  << (params.geometry_library == CircleNesting::Parameters::GeometryLibrary::Clipper ? "Clipper" : "CGAL")
                  << std::endl;

        if (fast_mode) {
            // ==========================
            // 极速模式：速度优先
            // 针对：约 300 个零件、平均 5 个顶点
            // ==========================

            // 直径优化：减少迭代次数，不使用二分，避免大量重复排样
            params.diameter_step_ratio = 0.01;              // 一次调整 1%
            params.min_diameter_ratio = 0.7;                // 不追求极限下界
            params.diameter_optimization_iterations = 20;   // 少量试探
            params.use_binary_search = false;               // 关闭二分查找

            // 紧凑化：迭代少一些，每一步走大一点
            params.compact_iterations = 100;                // 从 500 大幅降低
            params.compact_step_size = 0.004;               // 步长略放大
            params.compact_radius_search = 0.2;             // 搜索半径稍大一些
            params.use_simulated_annealing = false;         // 关闭模拟退火（省大量计算）

            // 候选点：候选位置数量大幅减少
            params.max_candidate_points = 800;              // 从 5000 降低
            params.use_radial_candidates = true;
            params.radial_layers = 10;                      // 从 30 降低
            params.angles_per_layer = 24;                   // 从 48 降低
            params.prioritize_compact_positions = true;

            // 放置策略：仍然保留智能排序和旋转尝试
            params.use_smart_ordering = true;
            params.try_all_rotations = true;

            // 调度算法：关闭多顺序搜索，避免重复完整排样
            params.use_scheduling = false;
            params.scheduling_attempts = 0;
            params.use_random_shuffle = false;
        } else {
            // ==========================
            // 默认高利用率模式：质量优先
            // ==========================
            params.diameter_step_ratio = 0.002;
            params.min_diameter_ratio = 0.5;
            params.diameter_optimization_iterations = 300;
            params.use_binary_search = true;
            params.compact_iterations = 5000;
            params.compact_step_size = 0.001;
            params.compact_radius_search = 0.15;
            params.use_simulated_annealing = true;
            params.sa_initial_temp = 0.2;
            params.sa_cooling_rate = 0.995;
            params.max_candidate_points = 20000;
            params.radial_layers = 80;
            params.angles_per_layer = 128;
            params.prioritize_compact_positions = true;
            params.use_smart_ordering = true;
            params.try_all_rotations = true;
            params.use_scheduling = true;
            params.scheduling_attempts = 50;
            params.use_random_shuffle = true;
            params.use_bound_evaluation = false;
        }

        // 单轮模式：不做“第二轮”类的重启搜索（多顺序/直径二分）。
        // 仅执行一次固定直径的放置 +（可选）紧凑化/局部修复，然后退出。
        

        // 阵列模式相关参数由 GUI 侧控制，这里保持默认：use_array_mode=false。
        circle_nesting.set_parameters(params);
        
        auto progress_wrapper = [&](const Layout& l) {
            double length = nesting::safe_to_double(l.sheets[0].diameter);
            double elapsed = static_cast<double>(clock() - start_clock) / CLOCKS_PER_SEC;

            // 始终优先使用 best_result，确保显示的是最佳利用率（避免显示下降）
            const std::vector<TransformedShape>* shapes = nullptr;
            double util = 0.0;
            
            if (!l.best_result.empty() && !l.best_result[0].empty()) {
                // 使用最佳结果，确保利用率不会下降
                shapes = &l.best_result[0];
                util = l.best_utilization;
            }
            else if (!l.sheet_parts.empty() && !l.sheet_parts[0].empty()) {
                // 如果没有 best_result，使用当前状态（但这种情况应该很少）
                shapes = &l.sheet_parts[0];
                // 实时计算当前布局的利用率（使用新的计算方法）
                try {
                    // 使用 CircleNesting 的计算方法（需要临时创建实例或直接计算）
                    // 这里简化处理：如果 best_utilization 有值就用它，否则计算
                    if (l.best_utilization > 0) {
                        util = l.best_utilization;
                    } else {
                        // 临时计算（简化版，实际应该调用 CircleNesting::calculate_utilization）
                        double total_area = nesting::safe_to_double(l.area);
                        double circle_area = Sheet::kPi * length * length / 4.0;
                        if (circle_area > 0) {
                            util = total_area / circle_area;
                        }
                    }
                }
                catch (...) {
                    util = 0.0;
                }
            }

            if (shapes) {
                ProgressHandler(Solution(length, util, elapsed, *shapes));
            }
        };
        
        // 执行优化
        // 目标利用率按用户要求：0.85（并受 max_time 时间预算约束）
        bool success = circle_nesting.optimize(1.1, requestQuit, progress_wrapper);
        
        // 无论成功与否，都确保报告最佳结果（避免停止时显示下降）
        // 如果 best_result 存在，说明有找到更好的解
        if (!layout.best_result.empty() && !layout.best_result[0].empty()) {
            // 使用最佳结果，确保不会下降
            progress_wrapper(layout);
        } else if (!success) {
            // 如果没有最佳结果且未成功，报告当前状态
            progress_wrapper(layout);
        }
    }

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
        const size_t circle_segments) {
        std::vector<nesting::Item> original_items;
        size_t size = polygons.size();
        for (size_t i = 0; i < size; i++) {
            original_items.emplace_back(items_quantity[i], polygons[i],
                items_rotations[i]);
        }
        FT area(0);
        for (auto& i : original_items) {
            area += geo::pwh_area(i.poly) * FT(i.quantity);
        }
        // 只支持圆形板材：sheet_height==0时，sheet_width为直径
        if (sheet_height != 0) {
            throw std::runtime_error("只支持圆形板材，请设置sheet_height=0");
        }
        
        std::vector<nesting::Sheet> original_sheets;
        original_sheets.push_back(
            nesting::Sheet::make_circle(FT(sheet_width), circle_segments));
        std::vector<nesting::Sheet> sheets;
        std::vector<nesting::Item> items;
        std::copy(original_items.begin(), original_items.end(),
            std::back_inserter(items));
        std::copy(original_sheets.begin(), original_sheets.end(),
            std::back_inserter(sheets));
        geo::preprocess(items, part_offset / 2, need_simplify);
        geo::preprocess(sheets[0], top_offset, left_offset, bottom_offset,
            right_offset);
        std::vector<nesting::Item> simplified_items;
        std::copy(items.begin(), items.end(), std::back_inserter(simplified_items));
        for (size_t i = 0; i < items.size(); i++) {
            auto& item = items[i];
            geo::offset_polygon(item.poly, part_offset / 2);
            auto first = item.poly.outer_boundary().vertices_begin();
            Transformation translate(CGAL::TRANSLATION,
                Vector_2(-first->x(), -first->y()));
            simplified_items[i].poly =
                geo::transform_polygon_with_holes(translate, simplified_items[i].poly);
        }
        nesting::Layout layout(items, sheets, area);
        return Preprocess(layout, original_items, simplified_items);
    }
}  // namespace nesting
