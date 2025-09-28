---
title: 间歇/版本差异的 DayMaterial.mdl 加载失败 (OmniUe4Translucent 关系解析)
category: 资源
order: 32
---

# DayMaterial.mdl / OmniUe4Translucent 间歇性加载报错

## 问题描述
在加载部分场景（`home_scenes`, `animation` 等同一来源数据）时，控制台出现如下 **偶发或特定版本才出现** 的 MDL 报错（示例截取）：
```
[Warning] [rtx.neuraylib.plugin] Loading MdlModule to DB (OmniUsdMdl) failed: ::Materials::DayMaterial
[Warning] [rtx.neuraylib.plugin] Loading transaction committed (this thread). MdlModule is NOT in the DB (OmniUsdMdl): ::Materials::DayMaterial
[Error]   [omni.usd] USD_MDL: LoadModule ... '::rtx::neuraylib::MdlModuleId' for '/.../Materials/DayMaterial.mdl' is Invalid
[Error]   [omni.usd] USD_MDL: GetSdrFromDiscoveryResult ... Module: '/.../Materials/DayMaterial.mdl' with version '1' not found in 'MdlModuleRegistry::ModuleDataMap'.
```
特点：
- 在 **Isaac Sim 4.1.0 不报错**，同样资源与环境在 **4.2.0 报错**（版本差异）。
- 不同同事机器有“有/无”差异（可能与缓存 / 环境变量 / 文件同时被操作有关）。
- 手动检查磁盘，`DayMaterial.mdl` 文件确实存在，且其引用的 `OmniUe4Translucent.mdl`（或同目录的 `OmniUe4Translucent` 派生材质）也在。 

## 给新手：DayMaterial.mdl 与 OmniUe4Translucent 是什么？
| 名称 | 角色 | 形象化理解 |
|------|------|------------|
| `DayMaterial.mdl` | 一个 MDL 模块 (Material Definition Language) 文件，内部定义一个或多个“白天/通用”材质函数 | 像一个“材质脚本”，被 USD 中的 Material prim 引用 |
| `OmniUe4Translucent.mdl` | 提供 UE4 风格半透明 / 透光材质实现的模块，被其它模块 `import` | 类似“基础半透明材质库” |

MDL 工作流简单回顾：
1. 解析 `module::identifier`（例如 `::Materials::DayMaterial`）。
2. 在 MDL 搜索路径里定位对应 `*.mdl` 文件。
3. 调用 MDL 编译器 (MDLC) + Neuray 接口编译成中间代码 / GPU shader。
4. 注册到模块数据库 (Module DB)。
如果任一阶段失败，后续引用节点生成 shader 时会出现上述错误。

## 可能的根本原因分层
| 层级 | 可能问题 | 线索 |
|------|----------|------|
| 文件系统 | 文件正在被同事写入 / 拷贝不完整 / 临时锁 | 同一时间“有人操作那个 folder” 的聊天记录提示并发写入 |
| 搜索路径 | 4.2.0 版本的默认 MDL 搜索路径顺序或集合改变 | 4.1.0 正常、4.2.0 失败 → 配置或扩展差异 |
| 缓存一致性 | 旧缓存引用了早期 hash，模块被视为无效 | 删除缓存后可能恢复 |
| 版本标签 | 模块内部 `mdl 1.x;` 或导出符号版本与引擎期望不匹配 | 日志里出现 `with version '1' not found` |
| 依赖链 | `DayMaterial.mdl` import 了 `OmniUe4Translucent`，后者先行失败导致前者级联失败 | 先搜索“第一个失败模块”日志条目 |
| 扩展 (Extension) | `omni.mdl` / `omni.usd` 插件版本升级，改了模块注册顺序或严格性 | 版本跨越后产生新校验逻辑 |
| 编码/换行 | 文件被 Windows 方式 (CRLF) 破坏或含 BOM | 用十六进制或文本查看器确认文件头 |

## 版本差异调查建议
| 步骤 | 4.1.0 | 4.2.0 | 说明 |
|------|-------|-------|------|
| 列出启用的相关扩展 | 记录 `omni.mdl` / `omni.usd` 版本号 | 同 | 比较 diff |
| 打印 MDL 搜索路径 | `mdl_iface.get_mdl_search_paths()` | 同 | 路径顺序与是否包含目标目录 |
| 校验文件哈希 | `sha256sum DayMaterial.mdl` | 同 | 确认文件完全一致 |
| 清缓存后重启 | 正常 | 仍失败? | 若 4.2.0 失败说明非偶发缓存 |
| 创建最小复现 | 引用仅 `DayMaterial` 的空场景 | 同复现? | 减少变量 |

## 快速诊断脚本（在 Kit Python Console）
```python
from omni.kit.app import get_app
app = get_app()
mdl = app.get_mdl_interface()
print('Search Paths:')
for p in mdl.get_mdl_search_paths():
    print('  -', p)

target = '::Materials::DayMaterial'
res = mdl.find_mdl_module(target)
print('Module find result:', res)

# 若失败，尝试直接加载文件路径
path_hint = '/absolute/path/to/Materials/DayMaterial.mdl'
ok = mdl.load_module_from_file(path_hint)
print('Fallback load from file ok?', ok)
```

## 排查步骤清单
1. 确认磁盘文件：大小、最近修改时间、哈希是否稳定（多人拷贝过程中可能出现 0 字节/部分内容）。
2. 比较两台机器 `MDL_SYSTEM_PATH` / 启动参数 `--/rtx/mdl/searchPaths+=` 是否一致。
3. 清除本地缓存：
   - Linux: `~/.cache/ov/` 下含 `mdl` / `shadercache` 目录。
   - Windows: `%LOCALAPPDATA%/ov/` 或 `%USERPROFILE%/.cache/ov/`（版本不同目录名略有差异）。
4. 临时将目标 `Materials` 目录置于搜索路径最前：
   ```bash
   ./isaac-sim.sh --/rtx/mdl/searchPaths="/ABS/PATH/Materials" --enable omni.mdl
   ```
5. 用最小 USD（仅一个 Material prim 引用该模块）测试：排除其它材质连带刷新影响。
6. 检查是否存在同名不同大小写的目录/文件（Windows 不区分大小写 → 模块注册冲突）。
7. 打开 `DayMaterial.mdl`：确认首行 `mdl 1.x;` 与文件编码；确认 import 行是否正确指向 `OmniUe4Translucent`（路径或模块名拼写）。
8. 如果 4.2.0 专属：在 Release Notes / `kit/exts/omni.mdl/` 里找 Changelog（可能引入更严格的模块验证）。

## 影响评估
| 场景影响 | 描述 | 风险 |
|----------|------|------|
| 仅该材质失败 | 显示 fallback（灰/默认） | 视觉不一致，渲染调优困难 |
| 级联失败（依赖树多模块） | 多个材质统一失效 | 大面积外观异常 |
| 不断重试 | 日志刷屏影响调试 | 轻微性能损耗 |
| Shader Pipeline 中断（少见） | 部分传感器/后处理失效 | 需要替换材质临时规避 |

## 临时缓解方案
| 方案 | 做法 | 代价 |
|------|------|------|
| 用占位 PBR 替换 | 简单 USD 材质 (Preview Surface) 绑定 | 失去自定义 effect |
| 手工复制模块到独立目录 | 将 `DayMaterial.mdl` 与依赖模块放入单独 `isolated_mdl/` 并首位搜索 | 需维护多份副本 |
| 强制 fallback 加载 | 检测失败后自动批量替换材质 prim | 替换脚本需回滚机制 |

## 永久修复策略建议
1. 制作一个“材质库完整性检查”脚本：扫描所有引用 `info:mdl:source`，验证对应模块可被 `find_mdl_module` 成功解析。
2. CI/提交钩子：对新增/改动的 `.mdl` 计算哈希并写入 manifest（部署端比对）。
3. 版本对齐：统一团队 Isaac Sim 版本，或为 4.2.0 单独验证 MDL 扩展兼容性。
4. 若确认是扩展缺陷：向 NVIDIA 反馈（包含最小复现：模块文件 + 最小 USD + 日志 + 版本号）。

## DayMaterial.mdl / OmniUe4Translucent 结构（典型示意）
（伪代码，仅帮助理解）
```mdl
mdl 1.7;
import ::OmniUe4Translucent as Ue4Trans;

export material DayMaterial(
    color base_color = color(1.0),
    float opacity = 1.0
) = Ue4Trans::TranslucentLayer(
    base_color: base_color,
    opacity: opacity
);
```
如果 `import ::OmniUe4Translucent` 失败，后续 `DayMaterial` 的导出自然失效 → 你在日志中看到顶层模块加载错误。

## 最小复现 USD 片段（Material prim）
```usda
def Material "DayMatTest"
{
    token info:mdl:source = "::Materials::DayMaterial"
}
```
加载该最小场景即可判定问题是否与其它资源无关。

## 验证 Checklist
| 项 | 期望 |
|----|------|
| 日志 | 不再出现 `Loading MdlModule... failed` / `Module ... not found` |
| 最小场景 | 单材质加载成功 |
| 原有复杂场景 | 显示恢复，材质参数可编辑 |
| 多次重启 | 结果稳定（消除“偶发”不确定性） |

## FAQ
**Q: 文件确实存在为何仍说 not found?** 可能是搜索路径未包含、或同名模块已注册失败缓存（需要清缓存再重试）。

**Q: 为什么版本升级后才出现?** 新版本可能更严格校验模块版本号 / 导入链；或者默认搜索路径顺序改变使“错误的同名旧拷贝”被优先解析。

**Q: 可以直接把内容复制到另一个名字绕过吗?** 可以临时验证是否语法本身损坏，但不是根治——必须确认实际模块链和路径。

**Q: 是否与权限有关?** 若目录只读且需要写 shader 缓存，有机会导致初始化流程异常；检查用户写权限。

## 截图
若你有同样报错，请把控制台截图保存为：`images/mdl-daymaterial-error.png`，并在此文档中引用：
```
![DayMaterial 加载错误](../images/mdl-daymaterial-error.png)
```
（目前仓库尚无此图片，放置后记得重新查看。）

> 总结：该问题通常是“模块存在但未被正常注册”——集中排查搜索路径、缓存、依赖导入顺序与版本差异。通过最小复现与接口脚本确认解析链，可快速定位是路径冲突、缓存污染还是版本回归问题，再决定是补路径、清缓存还是向上游反馈扩展回归。
