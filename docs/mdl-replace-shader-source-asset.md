---
title: 修改 / 替换 MDL Shader 的 info:mdl:sourceAsset 与 subIdentifier
category: 资源
order: 35
---

# 如何用 Python 修改一个 Shader 的 `info:mdl:sourceAsset` / `subIdentifier`（替换材质实现）

## 问题场景
在使用第三方（例如 GRUtopia）资产时，某个几何已经绑定到一个 Material；该 Material 里有一个 MDL Shader（通常类型 `UsdShade.Shader`，`id=UsdMdlShader` 或 ImplementationSource=sourceAsset），想把它引用的 **MDL 文件** 或 **该文件内导出的具体材质名称 (subIdentifier)** 换成另一个 MDL，而不是仅仅重新绑定一个新的 Material Prim。

典型需求：
* 批量“换皮肤”——保持 Material Prim 路径不变，内部换不同的 MDL 导出（便于脚本或已写死引用的上层逻辑）。
* 随机化材质：从同目录 `./Materials/*.mdl` 中随机挑一个 export。
* 替换不可直接重绑的（例如在 Reference / Instance 内层的 Material，想做 override）。

你可能最初尝试通过 `GetAttribute("info:mdl:sourceAsset")` 拿到一个 `SdfAssetPath`，但不确定如何写回；或者只找到“创建新 Material+Shader 再重新 binding”的方式（你给出的那段示例脚本）。本文总结“直接改 Shader 源” 与 “新建并 rebind” 两种方式的区别与推荐策略。

## 基础概念速览
| 术语 | 含义 | 备注 |
|------|------|------|
| `info:mdl:sourceAsset` | 一个 `SdfAssetPath`（包含 `path` 与 `resolvedPath`）指向 `.mdl` 文件 | 相对路径通常相对当前 Layer | 
| `info:mdl:sourceAsset:subIdentifier` | 该 MDL 文件里 export 的具体 material 名称 | 类似 `export material FancyWood` 中 `FancyWood` |
| ImplementationSource=sourceAsset | Shader 通过外部 MDL 文件提供实现 | 另两种常见：`sourceCode` (内嵌) 或 `id` (内置类型) |
| Material Prim 的 `material:binding` 关系 | 几何（Mesh / Xform）指向使用的 Material | 换材质可以改这个关系 |

## 两种换材质策略对比
| 策略 | 做法 | 优点 | 缺点 | 适用 |
|------|------|------|------|------|
| A. 新建 Material + Shader 后改 `material:binding` | 你的原脚本方式 | 不污染原 Material；可留旧版本备份 | 几何绑定路径改变；若有其它引用指向旧 Material 需同步 | 想保留历史或多版本共存 |
| B. 直接改现有 Shader 的 sourceAsset / subIdentifier | 原地修改 | 几何无感知；所有引用自动使用新外观 | 不可轻易回滚（需自己备份旧值） | 快速批量替换 / 保持指针稳定 |

> 如果资产来自 Reference 且层级被锁（在下层 Layer 定义），需要在“更强的 Layer / Session Layer”里做 override（见后文）。

## 方式 B（直接修改）最小示例
```python
from pxr import Usd, UsdShade, Sdf

stage = Usd.Stage.Open('scene.usd')

material_path = '/World/base/Looks/OldMaterial'  # 已存在的 Material Prim
mat = UsdShade.Material.Get(stage, Sdf.Path(material_path))
if not mat:
    raise RuntimeError('Material not found')

# 简化假设：Material 下只有一个 Shader 子 prim
shader_prims = [c for c in mat.GetPrim().GetChildren() if c.GetTypeName() == 'Shader']
if not shader_prims:
    raise RuntimeError('No Shader child under Material')
shader = UsdShade.Shader(shader_prims[0])

# 读取旧值（备份便于回滚）
old_asset = shader.GetAttribute('info:mdl:sourceAsset').Get()  # SdfAssetPath
old_sub  = shader.GetAttribute('info:mdl:sourceAsset:subIdentifier').Get()
print('OLD asset path =', old_asset.path, ' sub =', old_sub)

# 确保 ImplementationSource 是 sourceAsset
shader.CreateImplementationSourceAttr().Set(UsdShade.Tokens.sourceAsset)

# 写入新 MDL（相对路径示例）
new_mdl_rel = './Materials/WoodVariants.mdl'
new_sub_identifier = 'DarkOak'  # 该 mdl 文件中 export 的 material 名称
shader.SetSourceAsset(new_mdl_rel, 'mdl')
shader.SetSourceAssetSubIdentifier(new_sub_identifier, 'mdl')

stage.GetRootLayer().Save()
```

### 关键点解释
1. `shader.CreateImplementationSourceAttr()`：确保 ImplementationSource=sourceAsset，否则 `SetSourceAsset` 语义不生效。
2. `SetSourceAsset(path, 'mdl')` 第二个参数是“shader ID / source type” token，这里固定用 `'mdl'`。
3. `SetSourceAssetSubIdentifier(name, 'mdl')` 决定使用哪一个 `export material`。
4. 如果只换同一个 `.mdl` 内不同的 material，只改 subIdentifier 即可。
5. 保存：修改写入当前 EditTarget（默认 root layer）。如果原 Material 来自 Reference，下层不会被改，实际是一个 override 写在上层。

## 如何列出某个 MDL 文件内有哪些导出（subIdentifier）？
没有直接 USD API；可：
1. 解析 `.mdl` 文本（正则匹配 `export material`）。
2. 使用 MDL 接口（`mdl_iface.find_mdl_module()` 然后枚举导出符号；需 Kit Runtime，略复杂）。
3. 维护一个预生成 manifest（离线解析一次）。

简单正则示例（不处理所有语法，仅粗略）：
```python
import re, pathlib
def list_exports(mdl_file):
    text = pathlib.Path(mdl_file).read_text(encoding='utf-8', errors='ignore')
    return re.findall(r'export\s+material\s+([A-Za-z0-9_]+)', text)
print(list_exports('Materials/WoodVariants.mdl'))
```

## 批量随机替换例子（保留原 Material 不变路径）
```python
import os, random
from pxr import Usd, UsdShade, Sdf

def random_replace_material(stage, geom_prim_path, materials_dir='./Materials'):
    geom = stage.GetPrimAtPath(geom_prim_path)
    if not geom:
        return False
    # 取绑定 Material
    rel = geom.GetRelationship('material:binding')
    targets = rel.GetTargets()
    if not targets:
        return False
    mat_path = targets[0]
    mat = UsdShade.Material.Get(stage, mat_path)
    if not mat:
        return False
    # 找到子 Shader
    shader = None
    for c in mat.GetPrim().GetChildren():
        if c.GetTypeName() == 'Shader':
            shader = UsdShade.Shader(c); break
    if not shader:
        return False
    # 选择一个不同的 .mdl
    current_asset = shader.GetAttribute('info:mdl:sourceAsset').Get()
    current_path = current_asset.path if current_asset else ''
    candidates = [f for f in os.listdir(materials_dir) if f.endswith('.mdl') and f not in current_path]
    if not candidates:
        return False
    new_file = random.choice(candidates)
    # 简陋：导出名与文件名（去 .mdl）一致（若不一致需解析）
    sub = os.path.splitext(new_file)[0]
    shader.CreateImplementationSourceAttr().Set(UsdShade.Tokens.sourceAsset)
    shader.SetSourceAsset(os.path.join(materials_dir, new_file), 'mdl')
    shader.SetSourceAssetSubIdentifier(sub, 'mdl')
    return True

stage = Usd.Stage.Open('scene.usd')
random_replace_material(stage, '/World/base/obj_foo')
stage.GetRootLayer().Save()
```

## In-place 修改 vs 新建+Rebind 决策指南
| 需求 | 建议 |
|------|------|
| 保持外部引用稳定（脚本记住旧 Material 路径） | In-place 修改 |
| 希望多版本共存并可回滚 | 新建 + 重新绑定 |
| 想做 A/B 对比渲染 | 新建两个 material，切换 binding |
| 共享一组参数（相同 textures 调色） | In-place，避免复制 |

## 常见坑 & 解决
| 症状 | 原因 | 处理 |
|------|------|------|
| 修改后仍显示旧材质 | Viewer 缓存 / render 复用 shader | 切换渲染模式或强制刷新；保存+重新打开验证 |
| 设置 subIdentifier 后报“symbol not found” | 名称与 `.mdl` 内 export 不匹配 | 用正则列出 export 或手工打开 mdl 确认 |
| 改 `sourceAsset` 无效 | ImplementationSource 不是 `sourceAsset` | 调用 `CreateImplementationSourceAttr().Set(sourceAsset)` |
| 引用层不可写 | Material 来自 Referenced Layer | 使用 `with Usd.EditContext(stage, stage.GetSessionLayer())` 写 override |
| 想恢复旧值却忘记 | 未备份 | 修改前打印 / 记录到日志 / JSON |

## 在强层 (Session / 自建 Layer) 做 Override 示例
```python
from pxr import Usd, Sdf, UsdShade
stage = Usd.Stage.Open('scene.usd')
session = stage.GetSessionLayer()
with Usd.EditContext(stage, session):
    mat = UsdShade.Material.Get(stage, '/World/base/Looks/MatA')
    shader = UsdShade.Shader(mat.GetPrim().GetChildren()[0])
    shader.CreateImplementationSourceAttr().Set(UsdShade.Tokens.sourceAsset)
    shader.SetSourceAsset('./Materials/New.mdl', 'mdl')
    shader.SetSourceAssetSubIdentifier('NewExport', 'mdl')
stage.GetRootLayer().Save()
```
这样不修改底层资产文件，只在会话层覆盖，可随时丢弃。

## 什么时候不要 In-place 改？
* 同一 Material 需要在多个 View / 任务下表现不同（例如 Domain Randomization）。
* 下游有基于 `info:mdl:sourceAsset` 的缓存哈希：频繁更换会破坏缓存策略。
* 想保留原材质作回滚快照。

## 高级：同时调整参数
有些 MDL 导出有参数（在 UI 中显示 sliders）。在 In-place 更换后，可继续：
```python
param_attr = shader.GetInput('roughness')  # 取决于 mdl 导出的参数名
if param_attr:
    param_attr.Set(0.25)
```
若新旧导出参数名不同，需要先列出 `shader.GetInputs()`。

## 小型检测脚本：列出所有使用某个 MDL 的 Shader
```python
from pxr import Usd
stage = Usd.Stage.Open('scene.usd')
target_file = 'OldWood.mdl'
hits = []
for prim in stage.Traverse():
    if prim.GetTypeName() == 'Shader':
        attr = prim.GetAttribute('info:mdl:sourceAsset')
        if attr:
            ap = attr.Get()
            if ap and target_file in ap.path:
                hits.append(str(prim.GetPath()))
print('Shaders referencing', target_file, ':', hits)
```

## FAQ
**Q: 可以直接改 `GetAttribute(...).Set("xxx")` 吗？** 不建议；正确做法是用 `shader.SetSourceAsset()` 保证同时写入 `SdfAssetPath` 与 type token。

**Q: 想批量随机所有 Mesh 的材质？** 遍历 Stage：对每个有 material:binding 的 Prim 获取 Material→Shader，随机挑选并 In-place 改（或先克隆一个新的 Material 对象也行），注意去重防止所有对象都变一样。

**Q: `.mdl` 路径写相对还是绝对？** 推荐相对（相对于当前 USD Layer），可以随项目整体移动；绝对路径在跨机器共享时易失效。

**Q: subIdentifier 必须与文件同名吗？** 否；很多文件包含多个 exports。使用前列出或阅读文件。

## 总结
若只是“换同一个 Material 节点引用的 MDL 资源”，直接对 Shader 调用 `SetSourceAsset + SetSourceAssetSubIdentifier` 是最简洁且**不改变所有已绑定几何关系**的方案；新建 Material + 重绑适合多版本并存。记得：ImplementationSource 必须是 `sourceAsset`，修改前保存旧值便于回滚。
