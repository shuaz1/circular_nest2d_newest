#include "preprocess.h"

namespace nesting {
    namespace geo {
        void preprocess(std::vector<nesting::Item>& items,
            double offset,
            bool need_simplify) {
            for (auto& item : items) {
                auto& p = item.poly;
                auto& outer = p.outer_boundary();
                // 外轮廓需要是逆时针方向
                if (outer.is_clockwise_oriented()) {
                    outer.reverse_orientation();
                }
                for (auto& h : p.holes()) {
                    // 孔洞需要是顺时针方向
                    if (h.is_counterclockwise_oriented()) {
                        h.reverse_orientation();
                    }
                }
                // TODO 设置并捕获异常
                if (!is_valid_polygon_with_holes(p)) {
                    throw std::runtime_error("Input polygon is not valid");
                }
                // 简化多边形
                if (need_simplify) {
                    item.simplified = simplify(p);
                }
            }
        }

        void preprocess(Sheet& sheet,
            double top_offset,
            double left_offset,
            double bottom_offset,
            double right_offset) {
            // 仅支持圆形板材：height==0 作为标志
            // 根据四边偏置缩减直径，并同步更新圆形多边形
            double diameter_delta = -(top_offset + bottom_offset + left_offset + right_offset);
            double old_d = CGAL::to_double(sheet.diameter);
            double new_d = old_d + diameter_delta;
            if (new_d < 0.0) new_d = 0.0;

            sheet.diameter = FT(new_d);
            sheet.set_width(FT(new_d)); // 仅更新记录值
            sheet.height = FT(0);       // 保持为0，表示圆板

            // 重新构建圆形板材多边形
            size_t segs = sheet.segments;
            const double kPi = Sheet::kPi;
            geo::Polygon_2 boundary;
            boundary.clear();
            double cx = new_d / 2.0;
            double cy = new_d / 2.0;
            double r = new_d / 2.0;
            if (segs < 16) segs = 16;
            for (size_t i = 0; i < segs; ++i) {
                double theta = (2.0 * kPi * static_cast<double>(i)) / static_cast<double>(segs);
                double x = cx + r * std::cos(theta);
                double y = cy + r * std::sin(theta);
                boundary.push_back(geo::Point_2(FT(x), FT(y)));
            }
            if (boundary.is_clockwise_oriented()) {
                boundary.reverse_orientation();
            }
            sheet.sheet = geo::Polygon_with_holes_2(boundary);
        }
    }  // namespace geo
}  // namespace nesting