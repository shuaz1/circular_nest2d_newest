#pragma once
#include "basic.h"
#include <cmath>
#include <CGAL/number_utils.h>
namespace nesting {
    struct Sheet {
        // 现在只支持圆形板材，保留枚举是为了兼容旧代码，但不会再创建 Rectangle
        enum class ShapeType { Rectangle, Circle };
        static constexpr double kPi = 3.14159265358979323846;
        geo::FT width;
        geo::FT height;
        geo::FT diameter{ 0 };
        size_t segments{ 128 }; // number of segments used to approximate circle
        ShapeType type{ ShapeType::Circle };
        geo::Polygon_with_holes_2 sheet;
        // 构造函数：仅用于内部创建包围盒，外部请使用 make_circle
        explicit Sheet(geo::FT _width, geo::FT _height)
            : width(_width), height(_height), type(ShapeType::Circle) {
            // 默认构造为空，多边形由 make_circle 或 preprocess 重建
        }
        // 工厂方法：圆形（用正多边形近似），diameter为直径
        static Sheet make_circle(geo::FT d, size_t segments = 128) {
            Sheet s(d, d);
            s.type = ShapeType::Circle;
            s.diameter = d;
            s.segments = segments;
            auto& outer = s.sheet.outer_boundary();
            outer.clear();
            double r = CGAL::to_double(d) / 2.0;
            double cx = CGAL::to_double(d) / 2.0;
            double cy = CGAL::to_double(d) / 2.0;
            if (s.segments < 16) s.segments = 16; // 避免过少的段数
            for (size_t i = 0; i < s.segments; ++i) {
                double theta = (2.0 * kPi * static_cast<double>(i)) / static_cast<double>(s.segments);
                double x = cx + r * std::cos(theta);
                double y = cy + r * std::sin(theta);
                outer.push_back(geo::Point_2(geo::FT(x), geo::FT(y)));
            }
            return s;
        }
        // 将sheet的宽度变为_width（目前仅用于更新内部记录，不再修改多边形轮廓）
        inline void set_width(geo::FT _width) {
            width = _width;
        }
        // 将sheet的高度变为_height（圆板路径不会使用）
        inline void set_height(geo::FT _height) {
            height = _height;
        }
    };
}  // namespace nesting