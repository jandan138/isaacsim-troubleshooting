---
title: 物理烹饪失败：m_processResult == OperationResult::SUCCESS is false
category: 物理
order: 10
---

# 物理烹饪失败：`m_processResult == OperationResult::SUCCESS is false`

## 问题现象
- 加载某个场景（引用大量对象的聚合 USD）时终端持续出现：
  - `Warning [carb.ujitsoagent.plugin] UJITISO : Build storage validation failed ...`
  - `Warning [omni.physx.plugin] Failed to create convex decomposition mesh from cooked data!`（多次）
  - 最终：`[Error] [omni.physx.cooking.plugin] m_processResult == OperationResult::SUCCESS is false.`
- 进程进入“假死” / 长时间无响应：
  - 控制台无法用 `Ctrl + C` 中断，只能 kill 进程。
  - 后续简单批处理脚本（批量引用 USD）也被阻塞。
- 使用 Isaac Sim GUI 直接 File > Open 打开同一个 USD 可能偶尔成功，但二次启动或脚本批量加载不稳定：有时成功，有时报错。

## 触发背景
场景 USD 由大量 **Objaverse** 资产（OBJ → USD 转换后）逐个摆放再导出：
- 转换过程中未统一进行拓扑清理。
- 某些对象在最初放置时可见“正常”，但在再次加载聚合场景时触发 PhysX Mesh Cooking 失败。
- 通过将“疑似有问题”的对象加入黑名单可以降低复现概率，但仍存在后续随机触发。

## 核心概念速览（初学者友好）
| 名称 | 说明 |
|------|------|
| Cooking | PhysX 在仿真前把可渲染 Mesh 转换为高效的碰撞数据（凸包、分解、BVH 等）过程。 |
| Convex Decomposition | 将复杂凹多面体拆解为多个凸体以供快速碰撞检测。 |
| Convex Hull | 包含所有点的最小凸包；如果几何“太薄”或退化，可能 hull 退化为面/线。 |
| Hash / Metadata Validation | 内部缓存一致性检查，发现数据与记录不一致会告警并触发重新生成。 |

## 原因分析
> “OperationResult::SUCCESS is false” = PhysX 烹饪底层返回失败，Isaac Sim 外层断言触发。

常见导致失败的几类几何问题（结合经验与官方建议整理）：
1. 空/无效 Mesh：顶点数为 0，或只有顶点没有面（OBJ 转换时出现）。
2. 退化三角形 / 面积为 0：大量重合点或零长度边导致凸包算法失败。
3. 极端纵横比（Thin / Oblong）：厚度远小于长度/宽度，凸包体积近似为 0（如超薄布片、纸、平面）。
4. 非流形 / 自交：边界孔洞、重复面、反向法线、自相交拓扑导致分解算法无法稳定收敛。
5. 过度高分辨率：面数极高（>100k）并启用 convex decomposition，内存或时间超限 → 失败。
6. 单个资产在多次写入/转换过程中元数据（hash）不一致，引起缓存校验失败反复重烹饪，最终失败。
7. 不匹配的缩放：在 Stage 中对 Prim 应用极端非均匀缩放，放大了数值误差（例如 scale = (1000, 0.01, 1000)）。
8. 旧缓存 / 损坏缓存：`kit/cache/` 中的已缓存烹饪数据与当前 USD 不一致。

## 快速排查思路
| 排查项 | 命中指征 | 处理 |
|--------|---------|------|
| 是否特定资产导致 | 移除/黑名单后不再出现错误 | 针对该资产做拓扑清理或替换碰撞近似 |
| 是否缓存影响 | 删除 cache 后首次加载成功 | 加入定期清理或变更缓存策略 |
| 是否极薄几何 | 计算包围盒三轴比值 > 1000:1 | 给模型加厚或改用盒/胶囊碰撞 |
| 是否高面数 | 单 Mesh faces > 100k | 预先简化（decimate）或指定 primitive 碰撞 |
| 是否非流形 | 拓扑检查发现重复/孔洞 | Blender/meshlab 修复、重新导出 |

## 检查脚本（离线扫描 USD）
> 功能：遍历 Stage 中 Mesh，统计潜在风险：空面、退化、极端纵横比、高面数。
>
> 下方脚本加入“逐行”解释，适合初学者理解每一步做什么。你可以复制成 `scan_usd_geom.py` 运行。

```python
"""
scan_usd_geom.py
用法：python scan_usd_geom.py scene.usd > report.txt

检测项目：
 1. 空 Mesh / 没有点
 2. 没有面索引（总面顶点数为 0）
 3. 面顶点数量 < 3 的退化面
 4. 极端长宽高比例（判定为 THIN_GEOMETRY）
 5. 面数过高（> HIGH_FACE_THRESHOLD）

为什么需要这些检测？
  - PhysX 在做 convex hull / decomposition 时需要“有体积的”多面体；退化或超薄几何会让计算不稳定。
  - 过大的面数会延长烹饪时间，甚至内存超限返回失败。
  - 空或坏拓扑会直接导致烹饪返回 Error。
"""

from pxr import Usd, UsdGeom  # USD Python API：访问 Stage / Prim / Mesh 属性
import sys

# 如果一个物体的包围盒最长边 / 最短边 超过该阈值，就认为它“过薄”
THIN_RATIO = 800.0

# 面数阈值：如果一个 Mesh 的 face（多边形数）超过它，多半需要预先简化
HIGH_FACE_THRESHOLD = 100_000

def bbox_ratio(bounds):
    """给定三个轴向 (min,max) 范围，返回最长边 / 最短边。
    bounds: [(xmin,xmax),(ymin,ymax),(zmin,zmax)]
    如果最短边接近 0，则返回无穷大（表示极端薄）。"""
    lengths = [max(1e-9, b[1] - b[0]) for b in bounds]  # 防止 0 除错误
    longest = max(lengths)
    shortest = min(lengths)
    return longest / shortest if shortest > 0 else float('inf')

if len(sys.argv) < 2:
    print('Usage: python scan_usd_geom.py <scene.usd>')
    sys.exit(1)

# 打开目标 Stage（不会执行仿真，仅解析 USD 层）
stage = Usd.Stage.Open(sys.argv[1])
issues = []  # 收集 (路径, 问题类型)

for prim in stage.Traverse():  # 深度遍历整棵 Prim 树
    if not prim.IsA(UsdGeom.Mesh):  # 只关心 Mesh
        continue
    mesh = UsdGeom.Mesh(prim)
    pts = mesh.GetPointsAttr().Get()  # 顶点数组
    faces = mesh.GetFaceVertexCountsAttr().Get()  # 每个多边形的顶点数列表
    name = prim.GetPath().pathString  # Mesh 在 Stage 中的路径

    # 1. 空 Mesh：没有顶点
    if not pts:
        issues.append((name, 'EMPTY_MESH'))
        continue  # 后续检查看不到 bbox，直接跳过

    # 2. 完全没有面（有点没面）
    if sum(faces) == 0:
        issues.append((name, 'NO_FACE_INDICES'))

    # 3. 存在退化面：面顶点数 < 3
    if any(c < 3 for c in faces):
        issues.append((name, 'DEGENERATE_FACE_COUNT'))

    # 4. 计算世界包围盒判断是否“过薄”
    try:
        # ComputeWorldBound 可能失败（例如非常规 Boundable），因此放在 try 中
        bbox = UsdGeom.Boundable(prim).ComputeWorldBound(Usd.TimeCode.Default(), purpose1="default")
        minv = bbox.ComputeAlignedBox().GetMin()
        maxv = bbox.ComputeAlignedBox().GetMax()
        bounds = [(minv[i], maxv[i]) for i in range(3)]
        ratio = bbox_ratio(bounds)
        if ratio > THIN_RATIO:
            issues.append((name, f'THIN_GEOMETRY ratio={ratio:.1f}'))
    except Exception:
        # 某些 Prim 不支持或失败时忽略，不影响总体判定
        pass

    # 5. 面数过多（faces 列表长度即多边形数量）
    if len(faces) > HIGH_FACE_THRESHOLD:
        issues.append((name, f'HIGH_FACE_COUNT {len(faces)}'))

# 输出结果
if not issues:
    print('No obvious geometry issues found.')
else:
    for path, tag in issues:
        print('[ISSUE]', path, tag)
    print(f'Total issues: {len(issues)}')
```

使用：
```bash
python scan_usd_geom.py your_scene.usd > geom_report.txt
```

（可后续集成到 CI 过滤高风险资产。）

## 解决方法
### 方法一：预处理 / 修复 Mesh
1. Blender / MeshLab 导入 OBJ：
   - Remove Doubles（合并重复点）
   - Recalculate Normals (Outside)
   - Fill Holes / Make Manifold
   - Solidify（对过薄物体加厚）
2. Decimate / Quadric Simplification 将三角数降低（物理只需近似体积）。
3. 对纯平面（如海报、布片）使用盒 + 极薄厚度模型替代。

### 方法二：使用简单碰撞近似
在需要的 Mesh Prim 上设置属性（可以在 Isaac Sim 的 Property 面板 / 右键 Add Physics / 代码内设置）：
```
physics:approximation = "convexHull"
```
这会告诉烹饪系统：为该 Mesh 生成“单一凸包”作为碰撞体，而不是：
- 做复杂的凸分解（多个凸体组合）、
- 或直接把原三角网格作为动态碰撞（通常不允许 / 代价高）。

常见取值（不同版本可能略有差异，以当前 Isaac Sim / USD Physics 前缀为准）：
| 值 | 含义 | 适用对象 | 优点 | 风险/限制 |
|----|------|----------|------|-----------|
| convexHull | 单一凸包 | 大多数整体比较凸的物体 | 速度快，烹饪稳定 | 凹陷细节被抹平（可能出现“穿插错觉”） |
| convexDecomposition | 多个凸包组合 | 复杂凹形状需要较准体积 | 碰撞更贴合 | 烹饪时间长，失败概率更高 |
| none | 不自动生成近似 | 你会手动添加 Box / Capsule / Sphere / Mesh Collider | 完全可控 | 忘记加 collider 会导致无碰撞或运行时警告 |
| boundingSphere / boundingCube（若可用） | 使用包围体 | 粗略占位 / 临时调试 | 极快 | 碰撞不精准 |

为什么有时推荐写成：
```
physics:approximation = "convexHull"  # 或 "none" 然后手动添加简单 Collider
```
含义是：
1. 若模型形状比较“规整/近似凸”，直接 convexHull 即可。
2. 若模型极复杂（高多边形、很多孔洞）、经常触发 decomposition 失败，改成 `none`：
    - 关闭自动 hull / decomposition；
    - 你再手动创建一个简化的碰撞形状（例如一个或多个 Box / Capsule / 自己做的低模 Mesh）并放置在同一 Transform 下。

手动添加简单 Collider 的典型做法：
1. 在 `/World/Asset_X` 下新增一个 `Xform` 作为 `Colliders`。
2. 里面 Create > Physics > Shape（Box / Sphere / Capsule）。
3. 对每个 Shape 设置 `physics:collisionEnabled = true`，大小 / 平移匹配可视模型。
4. 将原始高模 Mesh 的 `physics:collisionEnabled` 设为 false（或 approximation=none）。

代码示例（Python 内部设置 convexHull）：
```python
from pxr import Usd, UsdGeom
stage = Usd.Stage.Open('scene.usd')
prim = stage.GetPrimAtPath('/World/Model_01')
prim.CreateAttribute('physics:approximation', 'token').Set('convexHull')
stage.GetRootLayer().Save()
```

使用 none + 自定义：
```python
prim.CreateAttribute('physics:approximation', 'token').Set('none')
# 然后脚本生成一个 box collider 作为替代
```

选择策略建议：
| 场景 | 推荐 | 原因 |
|------|------|------|
| 小型、规则、近似凸模型 | convexHull | 最快，失败概率低 |
| 复杂凹模型但需要较贴合 | convexDecomposition（若还稳定） | 精度更高 |
| 极复杂/退化多，烹饪常失败 | none + 手工简单形状 | 避免失败 & 可控 |
| 纯视觉远景资产（不交互） | none | 节省烹饪时间 |

调试技巧：
- 暂时把所有高风险资产（高面数/极薄）设置为 `none`，先验证场景其余部分能稳定加载；再逐个恢复。
- 对于角色/机械臂等需要精确接触区域的模型，用多个简单 Box 组合往往比凸分解更稳定也更快。

常见误区：
- 误以为“把 approximation 设为 none 仍会自动生成某种默认碰撞”——不会，你必须显式提供其他 Collider。
- 试图给纯平面布片做凸分解：Hull 会退化，引擎要么失败要么给出极差的体积。

> 简化总结：`convexHull` = 一键快速近似；`none` = 我要自己管碰撞；`convexDecomposition` = 更贴合但风险 / 成本高。

或把高复杂度资产作为 **视觉模型**，另建一个隐藏的低模（命名 `*_collider`）做碰撞，保持渲染与物理分离。

### 方法三：分层加载 & 二分定位
1. 先加载空场景。
2. 按批次引用子 USD（每批 N 个资产）。
3. 触发失败 → 对该批再分半测试，快速锁定 1~K 个可疑资产。

### 方法四：缓存清理
在 Linux：
```bash
rm -rf ~/.local/share/ov/pkg/isaac-sim-*/kit/cache/PhysX
```
重新启动后再加载，若显著好转 → 将“清缓存”加入周期性维护（或脚本检测 hash 冲突后自动清理）。

### 方法五：限制烹饪策略（选做）
在启动参数或设置中降低复杂度：
```
--/physics/cooking/convexDecomposition/enable=False
--/physics/cooking/forceCPU=True        # 调试用，便于区分 GPU 相关问题
```
（调试结束后恢复默认，以免影响真实碰撞精度。）

### 方法六：黑名单机制
维护 `bad_assets.txt`：当脚本批量引用时跳过名单中的资产；同时生成报告推动后续资产修复。

## 验证与回归
1. 运行扫描脚本，目标：无高风险标签或数量显著下降。
2. 分批加载日志中不再出现 `Failed to create convex decomposition`。
3. 全量加载后 5~10 次重复启动成功率稳定（100%）。
4. 开启仿真后可正常进入 Running，无卡住不响应。
5. 监控内存/显存：烹饪阶段峰值无异常暴涨。

## 附：最小化定位脚本示例（伪代码）
```python
assets = load_list('all_assets.txt')
suspects = []
while assets:
    batch = assets[:50]; assets = assets[50:]
    load_batch(batch)
    if cook_failed():
        suspects.extend(bisect_find_bad(batch))
unload_all(); print('Suspects:', suspects)
```

## 参考资料
- NVIDIA / PhysX 文档：Convex Hull & Cooking 指南
- USD 官方：Mesh 拓扑与有效性说明
- 经验实践：通过黑名单 + 几何清理 + 缓存清理三段式减少失败概率

> 总结：此错误 90% 来源于“不干净或不适合直接做凸分解的几何”。优先排查几何拓扑质量，其次再考虑缓存或引擎参数。
