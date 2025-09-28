---
title: 修改 Mesh 后出现大量 Hash Collision / Build storage validation failed 警告
category: 物理
order: 20
---

# 修改 Mesh Collider 后出现大量 Hash / Storage Validation 警告

## 问题现象
通过 OpenUSD (pxr) API 对一个已有场景/资产中的 Mesh 进行几何修改（增删顶点 / 重新生成形状）后，再在 Isaac Sim 中打开或引用该 USD，终端出现成片的警告：

```
[Warning] [carb.u...plugin] UJITISO : Build storage validation failed for  Storage status: 1
[Warning] [carb.u...plugin] UJITISO : Build storage validation failed ... Hash collision, data doesn't match at pos: XX : 0xAABB... - 0xCCDD...
[Warning] [omni.physx.plugin] Failed to create convex decomposition mesh from cooked data! Prim(...)
```

有时继续伴随：
```
[Error] [omni.physx.cooking.plugin] m_processResult == OperationResult::SUCCESS is false.
```
但也可能只是刷屏警告，场景最终仍然加载——只是仿真启动后性能/稳定性下降或部分碰撞体失效。

## 背景 & 触发方式
- 原始 USD 在 Isaac Sim 中曾经成功“烹饪”过（生成了碰撞用的 convex hull / decomposition 数据）。
- 之后使用 OpenUSD 脚本（未在 Isaac Sim 运行时）直接改写 Mesh 的拓扑：例如 `points` / `faceVertexCounts` / `faceVertexIndices`。
- 没有清除或更新与该 Mesh 相关的已缓存烹饪数据（内嵌或磁盘缓存）。
- 再次在 Isaac Sim 中加载：引擎试图复用旧缓存；哈希校验发现与当前几何不匹配 → 反复发出“hash collision / build storage validation failed”警告并尝试回退/重建。

## 初学者友好：到底在比对什么？
物理烹饪缓存常包含：
1. 输入几何摘要（点数量、面数量、AABB、哈希）
2. 输出数据（凸包点集、分解片段、BVH 索引）

加载时流程：
```
读取缓存 → 计算当前 Mesh 的哈希/统计 → 与缓存里记录值比较 → 
  一致：直接复用 → 快速进入仿真
  不一致：警告 + 尝试重新烹饪 → 写回缓存（或失败）
```
若旧缓存条目与“新几何”在某些字段碰巧 hash 冲突（或部分校验通过、部分失败），就会出现“Hash collision / meta difference at pos”式警告。

## 缓存可能存在的地方
| 形式 | 说明 | 典型路径 / 属性 | 是否随 USD 走 | 影响 |
|------|------|----------------|--------------|------|
| 运行时磁盘缓存 | Isaac Sim/Kit 启动后全局缓存 | `~/.local/share/ov/pkg/isaac-sim-*/kit/cache/PhysX/` (Linux) | 否 | 删除后会重新烹饪 |
| 内嵌 cooked 数据（可选） | 写进 USD 的自定义属性（例如以 `physx` / `physics` 前缀的 token / array blob） | 具体命名版本相关（如 `physxMesh:cooked` / `physxCollision:meshCookedData` 等） | 是 | 若不清理会与几何不一致 |
| 二级派生缓存 | Isaac Sim 扩展（如加速结构、JI(T) storage） | kit/exts/ 对应插件缓存 | 否 | 清理可强制重建 |

（命名会随版本变化，若需精确定位，可在修改前后保存两个 USD diff，搜 `cooked` / `physx`。）

## 主要成因归纳
| 类别 | 具体表现 | 说明 |
|------|----------|------|
| 几何拓扑修改未同步 | points 数量变化 | 缓存记录旧点数，校验失败 |
| 面索引重排 | faceVertexIndices 顺序变化 | 哈希不同，旧 cooked 数据结构不再匹配 |
| 非法/退化面引入 | 退化三角形 | 触发烹饪失败后反复尝试 |
| 极端非均匀缩放 | 后期在 Stage 上 scale 变形 | 尺寸与之前烹饪尺度差异过大 |
| 内嵌 cooked 属性残留 | USD 中残留旧二进制块 | 误判可复用导致警告 |

## 解决策略总览
| 场景 | 推荐操作 | 备注 |
|------|----------|------|
| 快速验证 | 完全删除 PhysX 磁盘缓存后重启再加载 | 观察警告是否消失 |
| 已内嵌 cooked 数据 | 脚本批量移除相关自定义属性 | 强制重新烹饪 |
| 频繁程序化修改几何 | 修改完成后调用一次“清理+烹饪”流程 | 缓存生命周期受控 |
| 复杂模型只做可视化 | 设置 `physics:approximation = none` 并提供独立简化碰撞体 | 避免重复烹饪风险 |
| 问题仍随机出现 | 排除退化几何 + 升级至匹配版本 | 确认不是版本 Bug |

## 操作步骤详解
### 步骤 1：清理磁盘缓存
Linux 示例：
```bash
rm -rf ~/.local/share/ov/pkg/isaac-sim-*/kit/cache/PhysX
```
Windows（PowerShell）：
```powershell
Remove-Item -Recurse -Force "$env:LOCALAPPDATA\ov\pkg\isaac-sim-*\kit\cache\PhysX"
```
重启 Isaac Sim 再加载场景观察是否仍有 hash 警告。

### 步骤 2：检测并移除内嵌 cooked / 旧属性
编写脚本扫描 Prim 属性名称中含 `cooked` / `physx` / `physxMesh`：
```python
from pxr import Usd
import sys

stage = Usd.Stage.Open(sys.argv[1])
to_clear = []
KEYWORDS = {"cooked", "physxMesh", "meshCooked", "cooking"}
for prim in stage.Traverse():
    for attr in prim.GetAttributes():
        name = attr.GetName()
        lower = name.lower()
        if any(k in lower for k in KEYWORDS):
            to_clear.append((prim.GetPath(), name))

print("Found candidate cooked attributes:")
for p, n in to_clear:
    print(p, n)

if len(sys.argv) > 2 and sys.argv[2] == "--remove":
    for p, n in to_clear:
        prim = stage.GetPrimAtPath(str(p))
        attr = prim.GetAttribute(n)
        prim.RemoveProperty(n)
    stage.GetRootLayer().Save()
    print("Removed cooked attributes.")
```
用法：
```bash
python scan_cooked_attrs.py scene.usd              # 只查看
python scan_cooked_attrs.py scene.usd --remove     # 真正删除（先备份！）
```

### 步骤 3：重新触发烹饪（可选）
在 Isaac Sim 启动后通过 Python：
```python
# 伪代码：访问目标 Mesh，读取一次 collision API，或暂时切换 approximation 再切回
from omni.isaac.core.utils.prims import set_prim_attribute
set_prim_attribute('/World/Model_A', 'physics:approximation', 'convexHull')
```
（具体接口会随版本变化；核心思想是强制引擎认为需重新生成。）

### 步骤 4：几何健康检查
运行先前文档中的 `scan_usd_geom.py` （或本仓库脚本）排除：空 Mesh、退化面、极薄几何。

### 步骤 5：避免未来再次发生
在“程序化几何修改 → 导出”流水线加入：
1. 移除旧 cooked 属性。
2. 记录一个 geometry version（自定义 metadata，如 `customData.geomVersion = <hash>`）。
3. 首次进入 Isaac Sim 后对比 `geomVersion` 与缓存，不一致则删缓存或强制重烹饪。

## 参考脚本：安全重写 Mesh 流程（简化示例）
```python
from pxr import Usd, UsdGeom, Gf

def rewrite_points(usdz_path, prim_path, scale=1.05):
    stage = Usd.Stage.Open(usdz_path)
    mesh = UsdGeom.Mesh.Get(stage, prim_path)
    pts = mesh.GetPointsAttr().Get()
    new_pts = [p * scale for p in pts]
    mesh.GetPointsAttr().Set(new_pts)
    # 标记一个自定义版本号（时间戳或 hash）
    prim = stage.GetPrimAtPath(prim_path)
    prim.SetCustomDataByKey('geomVersion', f'scale_adjust_{scale}')
    # 移除旧 cooked 属性（示例：若存在）
    for attr in list(prim.GetAttributes()):
        if 'cooked' in attr.GetName().lower():
            prim.RemoveProperty(attr.GetName())
    stage.GetRootLayer().Save()

# rewrite_points('scene.usd', '/World/Model_A')
```

## 验证与回归
| 验证项 | 期望 |
|--------|------|
| 首次重新加载日志 | 不再出现成片 hash collision / storage validation failed |
| 物理烹饪时间 | 正常（无无限等待/卡住） |
| 碰撞效果 | 物体有正确接触，未穿透/漂浮 |
| 重复加载多次 | 行为稳定一致 |

## FAQ
**Q: 只是警告但场景能跑，要不要管？** 仍建议清理；潜在隐藏问题：某些碰撞体实际未生效，后续仿真结果不可信。

**Q: 删除缓存后仍有警告？** 检查 USD 是否嵌入旧 cooked 属性；若有，移除后再试。

**Q: 是否可以关闭校验？** 不建议。校验是为了避免使用错误的几何数据导致不可预测的物理行为。

## 参考资料
- PhysX SDK Cooking 文档（Cook / Re-cook 条件）
- USD 自定义属性与 layer customData 机制
- 本仓库其它相关条目：物理烹饪失败、Mesh 几何质量检查

> 总结：警告本质是“几何与缓存不再一致”。策略：定位缓存来源 → 清理或移除嵌入数据 → 重新烹饪 → 建立修改流程约束防止复发。
