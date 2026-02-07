# Clipper 库集成指南

## 下载 Clipper 库

### 选项 1：Clipper2（推荐，需要 C++17）

1. **从 GitHub 下载**：
   - 访问：https://github.com/AngusJohnson/Clipper2
   - 点击 "Code" -> "Download ZIP"
   - 或者使用 Git：
     ```bash
     git clone https://github.com/AngusJohnson/Clipper2.git
     ```

2. **解压到项目目录**：
   - 将 Clipper2 文件夹放到项目根目录
   - 或者放到 `third_party/clipper2/` 目录

### 选项 2：Clipper 1.x（兼容性更好，不需要 C++17）

1. **从官网下载**：
   - 访问：http://www.angusj.com/delphi/clipper.php
   - 下载 "Clipper (C++)" 版本
   - 解压后找到 `clipper.hpp` 文件

2. **放置文件**：
   - 将 `clipper.hpp` 放到项目根目录
   - 或者放到 `third_party/clipper/` 目录

## 集成步骤

### 方法 1：使用 Clipper2（推荐）

1. **下载 Clipper2** 到项目目录：
   ```
   F:\git\auto\
   ├── clipper2\
   │   ├── clipper.h
   │   ├── clipper.core.h
   │   └── ... (其他文件)
   └── ...
   ```

2. **修改 `clipper_wrapper.h`**：
   - 找到 `#include "clipper2/clipper.h"`
   - 根据实际路径修改为：
     ```cpp
     #include "clipper2/clipper.h"  // 如果放在项目根目录
     // 或者
     #include "../clipper2/clipper.h"  // 如果放在其他位置
     ```

3. **启用 Clipper**：
   - 在 `clipper_wrapper.h` 中取消注释：
     ```cpp
     #define USE_CLIPPER
     ```
   - 或者在编译时添加宏定义：
     - Visual Studio: 项目属性 -> C/C++ -> 预处理器 -> 预处理器定义 -> 添加 `USE_CLIPPER`
     - CMake: `add_definitions(-DUSE_CLIPPER)`

### 方法 2：使用 Clipper 1.x

1. **下载 Clipper 1.x** 并放置 `clipper.hpp`：
   ```
   F:\git\auto\
   ├── clipper.hpp
   └── ...
   ```

2. **修改 `clipper_wrapper.h`**：
   - 将 `#include "clipper2/clipper.h"` 改为：
     ```cpp
     #include "clipper.hpp"
     ```

3. **修改 `clipper_wrapper.cpp`**：
   - 需要适配 Clipper 1.x 的 API（与 Clipper2 略有不同）
   - Clipper 1.x 使用 `Clipper` 类而不是 `Clipper64`
   - 使用 `IntPoint` 而不是 `Point64`

## 编译配置

### Visual Studio

1. **添加包含目录**（如果 Clipper 不在项目根目录）：
   - 项目属性 -> C/C++ -> 常规 -> 附加包含目录
   - 添加 Clipper 库的路径

2. **启用 C++17**（如果使用 Clipper2）：
   - 项目属性 -> C/C++ -> 语言 -> C++ 语言标准
   - 选择 "ISO C++17 标准" 或更高

### 验证集成

编译项目，如果没有错误，说明集成成功。

## 使用方式

在代码中，可以通过参数选择使用哪个库：

```cpp
// 在 Parameters 中设置
params.geometry_library = CircleNesting::Parameters::GeometryLibrary::Clipper;  // 使用 Clipper
// 或
params.geometry_library = CircleNesting::Parameters::GeometryLibrary::CGAL;     // 使用 CGAL（默认）
```

## 注意事项

1. **坐标精度**：
   - Clipper 使用整数坐标，需要缩放（当前实现使用 1e6 缩放因子）
   - CGAL 使用精确算术，精度更高

2. **性能**：
   - Clipper 通常比 CGAL 快 2-10 倍
   - 但对于复杂几何，CGAL 可能更稳定

3. **功能差异**：
   - Clipper 专注于布尔运算，功能较单一
   - CGAL 提供更多几何算法

4. **建议**：
   - 对于性能敏感的场景（如大量重叠检测），使用 Clipper
   - 对于需要高精度的场景，使用 CGAL
   - 可以混合使用：快速检测用 Clipper，精确计算用 CGAL

## 故障排除

### 编译错误：找不到 clipper.h

- 检查 Clipper 库的路径是否正确
- 检查包含目录设置

### 链接错误

- Clipper 是头文件库，不需要链接
- 如果使用 Clipper2，确保启用 C++17

### 运行时错误：精度问题

- 调整 `CLIPPER_SCALE` 的值（在 `clipper_wrapper.cpp` 中）
- 或使用 CGAL 进行精确计算

