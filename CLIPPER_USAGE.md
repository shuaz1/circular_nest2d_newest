# Clipper/CGAL 切换使用指南

## 当前实现状态

✅ 已实现通过参数切换 Clipper 和 CGAL 的功能

## 使用方式

### 方法 1：通过编译宏自动选择（推荐）

如果你已经定义了 `USE_CLIPPER` 宏，代码会自动使用 Clipper；否则使用 CGAL。

在 `interface.cpp` 中已经自动处理：
```cpp
#ifdef USE_CLIPPER
    params.geometry_library = CircleNesting::Parameters::GeometryLibrary::Clipper;
#else
    params.geometry_library = CircleNesting::Parameters::GeometryLibrary::CGAL;
#endif
```

### 方法 2：在代码中手动切换

如果你想在运行时动态切换，可以在设置参数时指定：

```cpp
CircleNesting::Parameters params;

// 使用 Clipper（快速）
params.geometry_library = CircleNesting::Parameters::GeometryLibrary::Clipper;

// 或使用 CGAL（精确，默认）
params.geometry_library = CircleNesting::Parameters::GeometryLibrary::CGAL;

circle_nesting.set_parameters(params);
```

## 已修改的函数

以下函数已支持 Clipper/CGAL 切换：

1. ✅ `calculate_utilization()` - 利用率计算
2. ✅ `is_valid_placement()` - 重叠检测和圆内检测
3. ✅ `calculate_waste_regions()` - 废料区域计算

## 注意事项

### Clipper2 API 兼容性

当前代码基于 Clipper2 的 API 编写。如果下载的是 Clipper 1.x，可能需要调整：

**Clipper2 使用：**
- `Clipper64`
- `Paths64`, `Path64`
- `Point64`
- `ClipType::Intersection`, `FillRule::EvenOdd`

**Clipper 1.x 使用：**
- `Clipper`
- `Paths`, `Path`
- `IntPoint`
- `ctIntersection`, `pftEvenOdd`

如果编译时出现 API 不匹配错误，请告诉我，我会帮你调整代码。

### 坐标精度

- Clipper 使用整数坐标，当前缩放因子为 `1e6`（保留 6 位小数精度）
- 如果精度不够，可以在 `clipper_wrapper.cpp` 中调整 `CLIPPER_SCALE` 的值

### 性能对比

- **Clipper**：通常快 2-10 倍，适合大量重叠检测
- **CGAL**：精度更高，适合复杂几何和精确计算

## 测试建议

1. 先用 CGAL 模式测试，确保功能正常
2. 然后切换到 Clipper 模式，对比性能和结果
3. 如果 Clipper 模式下结果有偏差，可以：
   - 增加 `CLIPPER_SCALE` 提高精度
   - 或混合使用：快速检测用 Clipper，精确计算用 CGAL

## 故障排除

### 编译错误：找不到 clipper.h

- 检查 Clipper 库路径是否正确
- 检查 `clipper_wrapper.h` 中的 include 路径

### 编译错误：API 不匹配

- 检查使用的是 Clipper2 还是 Clipper 1.x
- 告诉我具体的错误信息，我会帮你调整

### 运行时错误：精度问题

- 调整 `CLIPPER_SCALE` 的值
- 或切换回 CGAL 模式






