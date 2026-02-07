#pragma once
#include "layout.h"
#include "candidate_points.h"
#include <vector>
#include <functional>
#include <chrono>

namespace nesting {

    // 全局安全转换：用于将 CGAL 精确数值转换为 double（使用 interval，避免异常）
    double safe_to_double(const geo::FT& v);

    // 高利用率圆形排样核心算法
    class CircleNesting {
    public:
        explicit CircleNesting(Layout& layout);

        // 设置全局时间预算（秒）。0 表示不限制。
        void set_time_limit_seconds(size_t seconds);

        // 设置硬上限时间（秒）。0 表示不限制。
        void set_hard_time_limit_seconds(size_t seconds);

        // 设置质量门槛：在未达到 gate_utilization 前，忽略 soft time limit。
        void set_quality_gate(double gate_utilization);

        // 圆形排样参数
        struct Parameters {
            // 圆直径优化参数
            double diameter_step_ratio = 0.005;  // 每次直径调整的步长比例（0.5%，更精细）
            double min_diameter_ratio = 0.6;      // 最小直径相对于理论下界的比例
            size_t diameter_optimization_iterations = 100; // 直径优化迭代次数（增加）
            bool use_binary_search = true;       // 使用二分查找优化直径

            // 紧凑化参数
            size_t compact_iterations = 500;      // 局部紧凑化迭代次数（大幅增加）
            double compact_step_size = 0.002;     // 紧凑化步长（相对于直径，更小更精细）
            double compact_radius_search = 0.15;  // 紧凑化搜索半径（相对于直径）
            bool use_simulated_annealing = true; // 使用模拟退火优化
            double sa_initial_temp = 0.1;         // 模拟退火初始温度
            double sa_cooling_rate = 0.95;        // 模拟退火冷却率

            // NFP候选点参数
            size_t max_candidate_points = 2000;   // 最大候选点数（大幅增加）
            bool use_radial_candidates = true;    // 是否使用径向候选点生成
            size_t radial_layers = 20;            // 径向层数（增加）
            size_t angles_per_layer = 32;         // 每层角度数（增加）
            bool prioritize_compact_positions = true; // 优先选择紧凑位置

            // 放置策略参数
            bool use_smart_ordering = true;       // 使用智能排序（考虑形状复杂度）
            bool try_all_rotations = true;        // 为每个零件尝试所有旋转角度
            size_t max_rotation_attempts = 4;     // 最大旋转尝试次数

            // 旋转精度（度数）
            double rotation_precision = 1.0;      // 默认1度精度

            // 调度算法参数
            bool use_scheduling = true;           // 是否启用多顺序调度搜索
            size_t scheduling_attempts = 5;       // 调度尝试次数（不同顺序）
            bool use_random_shuffle = true;       // 是否使用随机洗牌生成新顺序

            // 阵列排样模式参数（可选）
            bool use_array_mode = false;          // 是否启用阵列排样模式（默认关闭，保持兼容）
            double array_pitch_x = 0.0;           // 阵列X方向间距（0表示自动：零件宽度+间隙）
            double array_pitch_y = 0.0;           // 阵列Y方向间距（0表示自动：零件高度+间隙）
            double array_margin = 0.0;            // 阵列最外圈距离圆边的安全距离

            // 废料最小化参数
            bool use_waste_minimization = true;    // 是否启用废料区域识别与优先填充
            double min_waste_area_ratio = 0.01;   // 最小废料区域面积比例（小于此值的废料区域忽略）
            size_t max_waste_candidates = 50;     // 每个废料区域最多生成的候选点数

            // 上界/下界评估参数
            bool use_bound_evaluation = true;      // 是否启用上界/下界差距评估
            double bound_gap_threshold = 0.01;     // 差距阈值（1%），当差距小于此值时提前停止优化
            bool use_improved_lower_bound = true;  // 是否使用改进的下界计算（考虑形状约束）
            bool use_hodograph_initial = false;    // 是否使用Hodograph方法生成快速初始解

            bool stop_after_first_stage = false;

            // 几何库选择参数
            enum class GeometryLibrary {
                CGAL,      // 使用 CGAL（精确但较慢）
                Clipper    // 使用 Clipper（快速但需要整数坐标）
            };
            GeometryLibrary geometry_library = GeometryLibrary::CGAL;  // 默认使用 CGAL
        };

        void set_parameters(const Parameters& params);

        // 执行高利用率圆形排样
        // 返回是否成功达到目标利用率（默认0.92，优秀级别）
        bool optimize(double target_utilization = 0.92, 
                     volatile bool* requestQuit = nullptr,
                     std::function<void(const Layout&)> progress_callback = nullptr);

        // 获取当前最优利用率
        double get_best_utilization() const { return best_utilization_; }

        // 获取当前圆直径
        double get_current_diameter() const;

    private:
        Layout& layout_;
        Parameters params_;
        double best_utilization_;
        double current_diameter_;

        bool have_validated_best_{ false };
        double last_validated_best_utilization_{ 0.0 };
        std::vector<std::vector<TransformedShape>> last_validated_best_result_;

        std::chrono::steady_clock::time_point next_cgal_validation_time_;

        bool use_time_limit_{ false };
        std::chrono::steady_clock::time_point deadline_;

        bool use_hard_time_limit_{ false };
        std::chrono::steady_clock::time_point hard_deadline_;

        bool use_quality_gate_{ false };
        double quality_gate_{ 0.0 };

        bool should_stop(volatile bool* requestQuit) const;

        bool cgal_validate_no_overlap_for_parts(const std::vector<TransformedShape>& parts,
                                                volatile bool* requestQuit) const;
        void maybe_cgal_validate_and_checkpoint_best(volatile bool* requestQuit);
        void rollback_to_last_validated_best();

        void local_flip_micro_shift_repair(volatile bool* requestQuit,
                                           std::function<void(const Layout&)>* progress_callback);

        // 计算理论最小直径（基于总面积）
        double calculate_theoretical_min_diameter() const;

        // 优化圆直径：从理论下界开始，逐步缩小直到无法放置
        bool optimize_diameter(double target_utilization, volatile bool* requestQuit,
                              std::function<void(const Layout&)> progress_callback);

        // 阵列排样模式：在固定直径下生成规则阵列布局
        bool optimize_array_mode(double target_utilization, volatile bool* requestQuit,
                                 std::function<void(const Layout&)> progress_callback);

        // 在给定直径下尝试放置所有零件
        bool try_place_all_parts(double diameter, volatile bool* requestQuit,
                                 std::function<void(const Layout&)>* progress_callback = nullptr);

        // 基于 NFP 的精确放置（针对圆形优化）
        // 增加 requestQuit 参数，便于在深度循环中及时响应“停止”请求
        bool place_shape_with_nfp(size_t shape_idx,
                                  const std::vector<size_t>& placed_indices, 
                                  bool try_all_rotations = true,
                                  volatile bool* requestQuit = nullptr,
                                  bool allow_waste_minimization = true);

        // 生成圆形优化的候选点
        std::vector<geo::Point_2> generate_circular_candidates(
            size_t shape_idx, 
            const std::vector<size_t>& placed_indices,
            const geo::Polygon_2& ifr);

        // 局部紧凑化：让已放置零件向空隙移动（增强版）
        void compact_layout(size_t iterations, volatile bool* requestQuit,
                           std::function<void(const Layout&)>* progress_callback = nullptr);

        // 智能排序：考虑面积、形状复杂度、长宽比
        std::vector<size_t> smart_order_parts() const;

        // 调度算法：尝试多种放置顺序，返回最优顺序
        std::vector<size_t> schedule_best_order(double diameter, volatile bool* requestQuit,
                                                 std::function<void(const Layout&)>* progress_callback);

        // 计算零件的形状复杂度（用于排序）
        double calculate_shape_complexity(size_t shape_idx) const;

        // 二分查找最优直径
        double binary_search_diameter(double min_diameter, double max_diameter, 
                                      double target_utilization, volatile bool* requestQuit,
                                      std::function<void(const Layout&)> progress_callback);

        // 计算当前布局的利用率
        double calculate_utilization() const;

        // 检查零件是否在圆内且无重叠
        bool is_valid_placement(size_t shape_idx, const std::vector<size_t>& placed_indices) const;

        // CGAL 模式下使用的简化版合法性检查（用于保证稳定性）
        bool is_valid_placement_cgal_simple(size_t shape_idx,
                                            const std::vector<size_t>& placed_indices) const;

        // 几何尺寸预检查：判断零件在当前圆直径下是否“无论如何都放不下”
        bool can_never_fit_in_circle(size_t shape_idx) const;

        // 更新圆直径并重建sheet
        void update_circle_diameter(double diameter);

        // 废料最小化相关函数
        // 计算当前布局的废料区域（圆 - 所有已放置零件的并集）
        std::vector<geo::Polygon_with_holes_2> calculate_waste_regions(
            const std::vector<size_t>& placed_indices) const;

        // 在废料区域生成候选点（优先在最大废料区域）
        std::vector<geo::Point_2> generate_waste_candidates(
            size_t shape_idx,
            const std::vector<size_t>& placed_indices,
            const std::vector<geo::Polygon_with_holes_2>& waste_regions) const;

        // 优先在废料区域放置零件
        bool place_shape_in_waste(size_t shape_idx, 
                                  const std::vector<size_t>& placed_indices,
                                  bool try_all_rotations);

        // 上界/下界评估相关函数
        // 计算改进的下界（考虑形状约束和旋转限制）
        double calculate_improved_lower_bound() const;

        // 计算上界/下界差距（返回差距百分比，0-1之间）
        double calculate_bound_gap() const;

        // 使用Hodograph方法生成快速初始解
        bool generate_hodograph_initial_solution(double diameter, volatile bool* requestQuit);
    };

} // namespace nesting

