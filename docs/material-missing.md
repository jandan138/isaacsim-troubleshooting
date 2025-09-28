# 材质缺失：物体显示粉色 / 灰色

## 问题现象
场景中导入的 USD / Mesh 模型出现粉色（Missing Material）或统一灰色，控制台/日志可能显示：
```
[Warning] Could not resolve material xxx.mdl
```
![材质缺失示例](../images/material-missing.png)

## 原因分析
- 材质引用路径不正确（相对路径 vs 绝对路径）。
- MDL / Texture 未被正确复制到项目资源目录。
- USD Stage 使用的渲染器与材质格式不兼容。
- 材质缓存或编译失败（临时文件损坏）。

## 解决方法
1. 检查 USD 中的材质路径：在 USD Viewer 中查看 `material:binding`。
2. 确认贴图文件存在：`albedo/baseColor/normal/roughness` 等贴图是否路径有效。
3. 若使用自定义 MDL，确保在 `omni.mdl` 搜索路径列表中：
   - 在 `kit/exts/omni.mdl/` 配置或启动参数中添加：
```
--/rtx/mdl/searchPaths+="/absolute/path/to/your_mdl"
```
4. 清理材质缓存：删除 `~/.cache/ov/` 下相关 shader / mdl 缓存再启动。
5. 切换渲染模式测试（View > Viewport > Renderers），确认是否特定渲染器问题。
6. 检查 log 中是否有权限 / I/O 读取失败。

## 验证与回归
- 模型应正确显示材质与纹理。
- 日志不再出现材质解析失败警告。
