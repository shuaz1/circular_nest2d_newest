#include "clipper_wrapper.h"
#include "algorithm.h"
#include <CGAL/number_utils.h>
#include <cmath>
#include <chrono>
#include <iostream>
#include <string>

namespace nesting {
    namespace geo {

        // Robust conversion from CGAL::FT to double using intervals
        static double safe_to_double(const geo::FT& v) {
            auto I = CGAL::to_interval(v);
            double a = I.first;
            double b = I.second;
            if (!std::isfinite(a) || !std::isfinite(b)) {
                return 0.0;
            }
            return 0.5 * (a + b);
        }

        GeometryOperations::Library GeometryOperations::current_library = 
#ifdef USE_CLIPPER
            GeometryOperations::Library::Clipper;
#else
            GeometryOperations::Library::CGAL;
#endif

#ifdef USE_CLIPPER
        // ============================================================================
        // Clipper 实现
        // ============================================================================

        struct ClipperPathsStats {
            size_t path_count = 0;
            size_t total_points = 0;
            int64_t max_abs_x = 0;
            int64_t max_abs_y = 0;
        };

        static ClipperPathsStats clipper_stats(const PathsType& paths) {
            ClipperPathsStats s;
            s.path_count = paths.size();
            for (const auto& p : paths) {
                s.total_points += p.size();
                for (const auto& pt : p) {
                    int64_t ax = pt.x >= 0 ? pt.x : -pt.x;
                    int64_t ay = pt.y >= 0 ? pt.y : -pt.y;
                    if (ax > s.max_abs_x) s.max_abs_x = ax;
                    if (ay > s.max_abs_y) s.max_abs_y = ay;
                }
            }
            return s;
        }

        PathsType GeometryOperations::cgal_to_clipper(const Polygon_with_holes_2& poly) {
            auto start = std::chrono::high_resolution_clock::now();
            std::string debugInfo = "cgal_to_clipper - ";

            PathsType result;
            
            try {
                // 转换外边界
                PathType outer;
                size_t vertexCount = 0;
                for (auto v = poly.outer_boundary().vertices_begin();
                     v != poly.outer_boundary().vertices_end(); ++v, ++vertexCount) {
                    double x = safe_to_double(v->x());
                    double y = safe_to_double(v->y());
                    outer.push_back(PointType(
                        static_cast<int64_t>(x * CLIPPER_SCALE),
                        static_cast<int64_t>(y * CLIPPER_SCALE)
                    ));
                }
                result.push_back(outer);

                // 转换孔（holes）
                int holeCount = 0;
                for (auto h = poly.holes_begin(); h != poly.holes_end(); ++h, ++holeCount) {
                PathType hole;
                for (auto v = h->vertices_begin(); v != h->vertices_end(); ++v) {
                    double x = safe_to_double(v->x());
                    double y = safe_to_double(v->y());
                    hole.push_back(PointType(
                        static_cast<int64_t>(x * CLIPPER_SCALE),
                        static_cast<int64_t>(y * CLIPPER_SCALE)
                    ));
                }
                result.push_back(hole);
                }
                
                debugInfo += "holes: " + std::to_string(holeCount) + 
                            ", outer points: " + std::to_string(vertexCount);
            }
            catch (const std::exception& e) {
                debugInfo += "Error: " + std::string(e.what());
                std::cerr << debugInfo << std::endl;
                throw;
            }

            auto end = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
            if (duration.count() > 100) {  // 如果耗时超过100毫秒
                std::cerr << debugInfo << " - took " << duration.count() << "ms" << std::endl;
            }

            return result;
        }

        std::vector<Polygon_with_holes_2> GeometryOperations::clipper_to_cgal(const PathsType& paths) {
            std::vector<Polygon_with_holes_2> result;
            
            if (paths.empty()) {
                return result;
            }

            try {
                // Clipper2 返回的路径集合中，每个路径都是独立的
                // 我们需要识别哪些是外边界（逆时针），哪些是孔（顺时针）
                // 使用 Area() 函数来判断：正面积 = 逆时针（外边界），负面积 = 顺时针（孔）
                std::vector<PathType> outer_paths;
                std::vector<std::vector<PathType>> holes_per_outer;
                
                for (const auto& path : paths) {
                if (path.size() < 3) continue; // 至少需要3个点
                
                try {
                    // 计算面积（使用 Clipper2 的 Area 函数）
                    // Clipper2: Area() 返回有符号面积，正数表示逆时针（外边界），负数表示顺时针（孔）
                    int64_t area_int = Clipper2Lib::Area(path);
                    double area = static_cast<double>(area_int) / (static_cast<double>(CLIPPER_SCALE) * CLIPPER_SCALE);
                    
                    if (area > 0) {
                        // 正面积 = 逆时针 = 外边界
                        outer_paths.push_back(path);
                        holes_per_outer.push_back(std::vector<PathType>());
                    } else {
                        // 负面积 = 顺时针 = 孔
                        // 找到最近的（包含它的）外边界
                        if (!outer_paths.empty()) {
                            // 简化：将孔添加到最后一个外边界
                            // 更精确的方法需要检查孔是否在外边界内部
                            holes_per_outer.back().push_back(path);
                        }
                    }
                } catch (...) {
                    // 如果单个路径转换失败，跳过它
                    continue;
                }
            }
            
            // 如果没有外边界，将所有路径都当作外边界（退化情况）
            if (outer_paths.empty()) {
                for (const auto& path : paths) {
                    if (path.size() < 3) continue;
                    try {
                        Polygon_2 outer;
                        for (const auto& pt : path) {
                            double x = static_cast<double>(pt.x) / CLIPPER_SCALE;
                            double y = static_cast<double>(pt.y) / CLIPPER_SCALE;
                            outer.push_back(Point_2(geo::FT(x), geo::FT(y)));
                        }
                        if (outer.size() >= 3) {
                            result.push_back(Polygon_with_holes_2(outer));
                        }
                    } catch (...) {
                        // 如果转换失败，跳过这个路径
                        continue;
                    }
                }
                return result;
            }
            
            // 转换每个外边界及其孔
            for (size_t i = 0; i < outer_paths.size(); ++i) {
                try {
                    const auto& outer_path = outer_paths[i];
                    Polygon_2 outer;
                    for (const auto& pt : outer_path) {
                        double x = static_cast<double>(pt.x) / CLIPPER_SCALE;
                        double y = static_cast<double>(pt.y) / CLIPPER_SCALE;
                        outer.push_back(Point_2(geo::FT(x), geo::FT(y)));
                    }
                    
                    if (outer.size() < 3) {
                        continue; // 至少需要3个点
                    }
                    
                    // 创建多边形（带孔）
                    Polygon_with_holes_2 pwh(outer);
                    
                    // 添加孔
                    for (const auto& hole_path : holes_per_outer[i]) {
                        try {
                            Polygon_2 hole;
                            for (const auto& pt : hole_path) {
                                double x = static_cast<double>(pt.x) / CLIPPER_SCALE;
                                double y = static_cast<double>(pt.y) / CLIPPER_SCALE;
                                hole.push_back(Point_2(geo::FT(x), geo::FT(y)));
                            }
                            if (hole.size() >= 3) {
                                // 确保孔是顺时针的（CGAL 要求）
                                if (hole.is_counterclockwise_oriented()) {
                                    hole.reverse_orientation();
                                }
                                pwh.add_hole(hole);
                            }
                        } catch (...) {
                            // 如果孔转换失败，跳过这个孔
                            continue;
                        }
                    }
                    
                    result.push_back(pwh);
                } catch (...) {
                    // 如果外边界转换失败，跳过这个多边形
                    continue;
                }
            }
            } catch (...) {
                // 如果整个转换过程失败，返回空列表
                result.clear();
            }

            return result;
        }

        BooleanResult GeometryOperations::intersection(
            const Polygon_with_holes_2& poly1,
            const Polygon_with_holes_2& poly2) {
            
            BooleanResult result;
            
            try {
                Paths64 paths1 = cgal_to_clipper(poly1);
                Paths64 paths2 = cgal_to_clipper(poly2);

                auto s1 = clipper_stats(paths1);
                auto s2 = clipper_stats(paths2);
                std::cerr << "[clipper] intersection Execute START"
                          << " subj(paths=" << s1.path_count << ",pts=" << s1.total_points
                          << ",maxAbsX=" << s1.max_abs_x << ",maxAbsY=" << s1.max_abs_y << ")"
                          << " clip(paths=" << s2.path_count << ",pts=" << s2.total_points
                          << ",maxAbsX=" << s2.max_abs_x << ",maxAbsY=" << s2.max_abs_y << ")"
                          << std::endl;
                auto t0 = std::chrono::high_resolution_clock::now();
                
                ClipperType clipper;
                clipper.AddSubject(paths1);
                clipper.AddClip(paths2);
                PathsType solution;
                
                if (clipper.Execute(ClipIntersection, FillEvenOdd, solution)) {
                    auto t1 = std::chrono::high_resolution_clock::now();
                    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
                    std::cerr << "[clipper] intersection Execute END " << ms << "ms"
                              << " solutionPaths=" << solution.size() << std::endl;
                    if (!solution.empty()) {
                        result.polygons = clipper_to_cgal(solution);
                        // 如果转换后为空，说明转换失败，但 Clipper 执行成功，可能真的没有交集
                        result.is_empty = result.polygons.empty();
                        
                        // 计算总面积
                        for (const auto& poly : result.polygons) {
                            result.total_area += safe_to_double(pwh_area(poly));
                        }
                    } else {
                        // 空结果，没有交集
                        result.is_empty = true;
                        result.total_area = 0.0;
                        result.polygons.clear();
                    }
                } else {
                    auto t1 = std::chrono::high_resolution_clock::now();
                    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
                    std::cerr << "[clipper] intersection Execute FAIL " << ms << "ms" << std::endl;
                    // Clipper 执行失败，返回空结果（保守策略：假设没有交集，避免误判）
                    result.is_empty = true;
                    result.total_area = 0.0;
                    result.polygons.clear();
                }
            } catch (...) {
                result.is_empty = true;
            }
            
            return result;
        }

        BooleanResult GeometryOperations::join(
            const Polygon_with_holes_2& poly1,
            const Polygon_with_holes_2& poly2) {
            
            BooleanResult result;
            
            try {
                PathsType paths1 = cgal_to_clipper(poly1);
                PathsType paths2 = cgal_to_clipper(poly2);

                auto s1 = clipper_stats(paths1);
                auto s2 = clipper_stats(paths2);
                std::cerr << "[clipper] union Execute START"
                          << " subj(paths=" << s1.path_count << ",pts=" << s1.total_points
                          << ",maxAbsX=" << s1.max_abs_x << ",maxAbsY=" << s1.max_abs_y << ")"
                          << " clip(paths=" << s2.path_count << ",pts=" << s2.total_points
                          << ",maxAbsX=" << s2.max_abs_x << ",maxAbsY=" << s2.max_abs_y << ")"
                          << std::endl;
                auto t0 = std::chrono::high_resolution_clock::now();
                
                ClipperType clipper;
                clipper.AddSubject(paths1);
                clipper.AddClip(paths2);
                PathsType solution;
                
                if (clipper.Execute(ClipUnion, FillEvenOdd, solution)) {
                    auto t1 = std::chrono::high_resolution_clock::now();
                    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
                    std::cerr << "[clipper] union Execute END " << ms << "ms"
                              << " solutionPaths=" << solution.size() << std::endl;
                    if (!solution.empty()) {
                        result.polygons = clipper_to_cgal(solution);
                        result.is_empty = result.polygons.empty();
                        
                        // 计算总面积
                        for (const auto& poly : result.polygons) {
                            result.total_area += safe_to_double(pwh_area(poly));
                        }
                    } else {
                        // 空结果
                        result.is_empty = true;
                        result.total_area = 0.0;
                    }
                } else {
                    auto t1 = std::chrono::high_resolution_clock::now();
                    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
                    std::cerr << "[clipper] union Execute FAIL " << ms << "ms" << std::endl;
                    // Clipper 执行失败，返回空结果
                    result.is_empty = true;
                    result.total_area = 0.0;
                }
            } catch (...) {
                result.is_empty = true;
            }
            
            return result;
        }

        BooleanResult GeometryOperations::difference(
            const Polygon_with_holes_2& poly1,
            const Polygon_with_holes_2& poly2) {
            
            BooleanResult result;
            
            try {
                PathsType paths1 = cgal_to_clipper(poly1);
                PathsType paths2 = cgal_to_clipper(poly2);

                auto s1 = clipper_stats(paths1);
                auto s2 = clipper_stats(paths2);
                std::cerr << "[clipper] diff Execute START"
                          << " subj(paths=" << s1.path_count << ",pts=" << s1.total_points
                          << ",maxAbsX=" << s1.max_abs_x << ",maxAbsY=" << s1.max_abs_y << ")"
                          << " clip(paths=" << s2.path_count << ",pts=" << s2.total_points
                          << ",maxAbsX=" << s2.max_abs_x << ",maxAbsY=" << s2.max_abs_y << ")"
                          << std::endl;
                auto t0 = std::chrono::high_resolution_clock::now();
                
                ClipperType clipper;
                clipper.AddSubject(paths1);
                clipper.AddClip(paths2);
                PathsType solution;
                
                if (clipper.Execute(ClipDifference, FillEvenOdd, solution)) {
                    auto t1 = std::chrono::high_resolution_clock::now();
                    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
                    std::cerr << "[clipper] diff Execute END " << ms << "ms"
                              << " solutionPaths=" << solution.size() << std::endl;
                    if (!solution.empty()) {
                        result.polygons = clipper_to_cgal(solution);
                        result.is_empty = result.polygons.empty();
                        
                        // 计算总面积
                        for (const auto& poly : result.polygons) {
                            result.total_area += safe_to_double(pwh_area(poly));
                        }
                    } else {
                        // 空结果
                        result.is_empty = true;
                        result.total_area = 0.0;
                    }
                } else {
                    auto t1 = std::chrono::high_resolution_clock::now();
                    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
                    std::cerr << "[clipper] diff Execute FAIL " << ms << "ms" << std::endl;
                    // Clipper 执行失败，返回空结果
                    result.is_empty = true;
                    result.total_area = 0.0;
                }
            } catch (...) {
                result.is_empty = true;
            }
            
            return result;
        }

        bool GeometryOperations::do_intersect(
            const Polygon_with_holes_2& poly1,
            const Polygon_with_holes_2& poly2) {
            
            try {
                PathsType paths1 = cgal_to_clipper(poly1);
                PathsType paths2 = cgal_to_clipper(poly2);

                auto s1 = clipper_stats(paths1);
                auto s2 = clipper_stats(paths2);
                std::cerr << "[clipper] do_intersect Execute START"
                          << " subj(paths=" << s1.path_count << ",pts=" << s1.total_points
                          << ",maxAbsX=" << s1.max_abs_x << ",maxAbsY=" << s1.max_abs_y << ")"
                          << " clip(paths=" << s2.path_count << ",pts=" << s2.total_points
                          << ",maxAbsX=" << s2.max_abs_x << ",maxAbsY=" << s2.max_abs_y << ")"
                          << std::endl;
                auto t0 = std::chrono::high_resolution_clock::now();
                
                ClipperType clipper;
                clipper.AddSubject(paths1);
                clipper.AddClip(paths2);
                PathsType solution;
                
                bool ok = clipper.Execute(ClipIntersection, FillEvenOdd, solution);
                auto t1 = std::chrono::high_resolution_clock::now();
                auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count();
                std::cerr << "[clipper] do_intersect Execute END " << ms << "ms"
                          << " ok=" << (ok ? 1 : 0) << " solutionPaths=" << solution.size() << std::endl;
                return ok && !solution.empty();
            } catch (...) {
                return false;
            }
        }

        double GeometryOperations::area(const Polygon_with_holes_2& poly) {
            try {
                // 对于带孔的多边形，应该使用 CGAL 的精确计算
                // Clipper 的面积计算可能不准确，特别是对于带孔的多边形
                // 回退到 CGAL 以确保精度
                return safe_to_double(pwh_area(poly));
            } catch (...) {
                // 如果 CGAL 也失败，尝试使用 Clipper
                try {
                    PathsType paths = cgal_to_clipper(poly);
                    if (paths.empty()) return 0.0;
                    
                    // 使用 Clipper 的面积计算
                    // 对于外边界，面积为正；对于孔，面积为负
                    // 总面积 = 外边界面积 - 所有孔的面积
                    double total_area = 0.0;
                    bool has_outer = false;
                    
                    for (const auto& path : paths) {
                        if (path.size() < 3) continue;
                        // Clipper2: Area() 函数返回有符号面积
                        int64_t area_int = Area(path);
                        double area = static_cast<double>(area_int) / (static_cast<double>(CLIPPER_SCALE) * CLIPPER_SCALE);
                        
                        if (area > 0) {
                            // 外边界（逆时针）
                            total_area += area;
                            has_outer = true;
                        } else {
                            // 孔（顺时针），减去其面积
                            total_area -= std::abs(area);
                        }
                    }
                    
                    // 如果没有外边界，返回0
                    if (!has_outer) {
                        return 0.0;
                    }
                    
                    return std::max(0.0, total_area);
                } catch (...) {
                    return 0.0;
                }
            }
        }

#else
        // ============================================================================
        // CGAL 实现（默认）
        // ============================================================================

        BooleanResult GeometryOperations::intersection(
            const Polygon_with_holes_2& poly1,
            const Polygon_with_holes_2& poly2) {
            
            BooleanResult result;
            
            try {
                std::vector<Polygon_with_holes_2> inters;
                CGAL::intersection(poly1, poly2, std::back_inserter(inters));
                
                result.polygons = inters;
                result.is_empty = inters.empty();
                
                for (const auto& poly : inters) {
                    result.total_area += safe_to_double(pwh_area(poly));
                }
            } catch (...) {
                result.is_empty = true;
            }
            
            return result;
        }

        BooleanResult GeometryOperations::join(
            const Polygon_with_holes_2& poly1,
            const Polygon_with_holes_2& poly2) {
            
            BooleanResult result;
            
            try {
                geo::Polygon_set_2 ps(geo::traits);
                ps.insert(poly1.outer_boundary());
                ps.insert(poly2.outer_boundary());
                
                std::vector<Polygon_with_holes_2> unions;
                ps.polygons_with_holes(std::back_inserter(unions));
                
                result.polygons = unions;
                result.is_empty = unions.empty();
                
                for (const auto& poly : unions) {
                    result.total_area += safe_to_double(pwh_area(poly));
                }
            } catch (...) {
                result.is_empty = true;
            }
            
            return result;
        }

        BooleanResult GeometryOperations::difference(
            const Polygon_with_holes_2& poly1,
            const Polygon_with_holes_2& poly2) {
            
            BooleanResult result;
            
            try {
                geo::Polygon_set_2 ps(geo::traits);
                ps.insert(poly1.outer_boundary());
                ps.difference(poly2.outer_boundary());
                
                std::vector<Polygon_with_holes_2> diffs;
                ps.polygons_with_holes(std::back_inserter(diffs));
                
                result.polygons = diffs;
                result.is_empty = diffs.empty();
                
                for (const auto& poly : diffs) {
                    result.total_area += safe_to_double(pwh_area(poly));
                }
            } catch (...) {
                result.is_empty = true;
            }
            
            return result;
        }

        bool GeometryOperations::do_intersect(
            const Polygon_with_holes_2& poly1,
            const Polygon_with_holes_2& poly2) {
            
            try {
                return CGAL::do_intersect(poly1, poly2);
            } catch (...) {
                return false;
            }
        }

        double GeometryOperations::area(const Polygon_with_holes_2& poly) {
            return safe_to_double(pwh_area(poly));
        }

#endif

    } // namespace geo
} // namespace nesting

