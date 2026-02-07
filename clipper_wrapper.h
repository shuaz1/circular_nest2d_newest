#pragma once

// Clipper 包装层：提供统一的接口，支持 CGAL 和 Clipper 之间的切换
// 注意：使用 Clipper 需要先下载并集成 Clipper 库

#include "basic.h"
#include <vector>

// 如果定义了 USE_CLIPPER，则使用 Clipper；否则使用 CGAL
// 可以通过编译选项或取消注释下面的行来启用 Clipper
// #define USE_CLIPPER

#ifdef USE_CLIPPER
    // Clipper 库的头文件（需要先下载 Clipper 库）
    // Clipper 1.x: #include "clipper.hpp"
    // Clipper2: #include "clipper2/clipper.h"
    // 这里假设使用 Clipper2，如果使用 Clipper 1.x，需要修改下面的代码
    #include <clipper2/clipper.h>
#else
    // 使用 CGAL（默认）
    #include <CGAL/boolean_set_operations_2.h>
#endif

namespace nesting {
    namespace geo {

#ifdef USE_CLIPPER
        // Clipper2 类型定义（在命名空间内，避免冲突）
        using namespace Clipper2Lib;
        using ClipperType = Clipper64;
        using PathsType = Paths64;
        using PathType = Path64;
        using PointType = Point64;
        constexpr auto ClipIntersection = ClipType::Intersection;
        constexpr auto ClipUnion = ClipType::Union;
        constexpr auto ClipDifference = ClipType::Difference;
        constexpr auto FillEvenOdd = FillRule::EvenOdd;
#endif

        // 多边形布尔运算结果
        struct BooleanResult {
            std::vector<Polygon_with_holes_2> polygons;
            double total_area = 0.0;
            bool is_empty = true;
        };

        // 统一的布尔运算接口
        class GeometryOperations {
        public:
            enum class Library {
                CGAL,
                Clipper
            };

            static Library current_library;

            // 交集运算
            static BooleanResult intersection(
                const Polygon_with_holes_2& poly1,
                const Polygon_with_holes_2& poly2
            );

            // 并集运算
            static BooleanResult join(
                const Polygon_with_holes_2& poly1,
                const Polygon_with_holes_2& poly2
            );

            // 差集运算
            static BooleanResult difference(
                const Polygon_with_holes_2& poly1,
                const Polygon_with_holes_2& poly2
            );

            // 检查是否相交
            static bool do_intersect(
                const Polygon_with_holes_2& poly1,
                const Polygon_with_holes_2& poly2
            );

            // 计算面积
            static double area(const Polygon_with_holes_2& poly);

        private:
#ifdef USE_CLIPPER
            // Clipper 辅助函数：将 CGAL 多边形转换为 Clipper 路径
            static PathsType cgal_to_clipper(const Polygon_with_holes_2& poly);
            
            // Clipper 辅助函数：将 Clipper 路径转换为 CGAL 多边形
            static std::vector<Polygon_with_holes_2> clipper_to_cgal(const PathsType& paths);
            
            // 坐标缩放因子（Clipper 使用整数坐标，需要缩放）
            static constexpr int64_t CLIPPER_SCALE = 1000000; // 1e6，保留6位小数精度
#endif
        };

    } // namespace geo
} // namespace nesting

