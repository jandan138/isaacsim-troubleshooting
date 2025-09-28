---
title: 批量为地板与家具绑定 Collider / 刚体（脚本自动化实践）
category: 物理
order: 40
---

# 批量为地板与家具绑定 Collider / 刚体

## 问题描述
在原始大场景 USD（包含多种家具 / 建筑结构）中，需要快速为“可交互的家具”添加刚体与碰撞形状，同时为“地板/静态结构”添加静态碰撞，使仿真里物体可以正确落地 / 碰撞。手动选中一个个 Prim 设置极其低效且容易遗漏，因此需要一个离线脚本自动遍历并批量处理。

## 使用场景
- 大量从扫描 / 第三方数据集（如 Objaverse、室内合成集）转换来的 USD，没有 physics 相关属性。
- 需要区分：
  - 地板 / 墙体 / 门框 等“静态环境”（Static Collider）。
  - 桌椅、柜子、家电、箱子、可推动物体等“动态对象”（Rigid Body + Collider）。

## 实现方案概览
1. 启动一个最小的 `SimulationApp`（因为 `omni.physx.scripts.utils` 依赖 Kit 环境）。
2. 打开目标 Stage。
3. （可选）统一缩放或补救错误缩放。
4. 遍历 Prim：
   - 通过名称 + 包围盒尺寸启发式判断类型。
   - 为动态物体调用 `utils.setRigidBody(prim, "convexDecomposition", False)`。
   - 为静态地板只添加 Collider（不设刚体或设为 Kinematic / `rigid_body_enabled=False`）。
5. 针对凸分解的参数（最大 hull 数、最小厚度等）按需设置。
6. 可以为较重或需要物理效果的对象绑定密度（`physicsUtils.add_density`）。
7. 保存到新文件（避免覆盖原始资产）。
8. 输出处理统计（多少个动态 / 静态 / 跳过）。

## 详细脚本（带超详细注释）
> 若你只想直接用，可复制到 `scripts/batch_add_colliders.py`。脚本假设使用 Isaac Sim Python 环境运行。

```python
#!/usr/bin/env python3
"""
批量为场景中的家具 / 地板添加 Collider 与刚体示例。

用法：
    ./python.sh batch_add_colliders.py \
        --input /path/to/scene.usd \
        --output /path/to/scene_with_physics.usd \
        --scale 0.001 \
        --dry-run

参数说明：
    --input / --output : 输入与输出 USD 路径
    --scale            : 可选，全局缩放（用于外部导入单位差异，例如 mm→m）
    --dry-run          : 只打印计划，不写回文件
    --density          : 为动态物体设置密度（默认 1000 kg/m^3，可按需要调整）
    --max-hulls        : 凸分解最大 hull 数 (默认 64)
    --filter           : 只处理路径包含该子串的 Prim（调试用）

启发式分类策略（可按需扩展）：
    1) 名称匹配 floor|ground|tile → 静态地板
    2) 名称匹配 wall|door(frame)?|window(frame)? → 静态结构
    3) 名称匹配 table|chair|sofa|bed|cabinet|desk|drawer|shelf|microwave|fridge|cooker 等 → 动态家具
    4) 体素尺寸判断：高度很小且面积大 → 可能是地板

限制：
    - 复杂 Mesh 采用 convexDecomposition 可能耗时；若失败可改为 convexHull 或手动 low-poly 碰撞。
    - 本脚本不做退化几何清理（可结合本仓库其它检查脚本）。
"""

from omni.isaac.kit import SimulationApp  # 必须最先导入并初始化
import argparse
import re
from pxr import Usd, UsdGeom, Gf
from omni.physx.scripts import utils as physx_utils, physicsUtils

simulation_app = SimulationApp({
    "headless": True,
    "width": 1280,
    "height": 720,
})

STATIC_NAME_PATTERN = re.compile(r"(floor|ground|tile|wall|door|doorframe|window|windowframe)", re.IGNORECASE)
DYNAMIC_NAME_PATTERN = re.compile(r"(table|chair|sofa|bed|cabinet|desk|drawer|shelf|stool|bench|microwave|fridge|refrigerator|cooker|oven|washer|dryer|tv|monitor|lamp|trash|box)", re.IGNORECASE)

def classify(prim):
    """根据名称与包围盒启发式分类: return 'static' | 'dynamic' | 'skip'"""
    name = prim.GetName()
    path = prim.GetPath().pathString
    # 只对 Mesh 或 Xform 容器处理
    t = prim.GetTypeName()
    if t not in ("Mesh", "Xform", "Scope"):
        return 'skip'
    low = name.lower()
    if STATIC_NAME_PATTERN.search(low):
        return 'static'
    if DYNAMIC_NAME_PATTERN.search(low):
        return 'dynamic'
    # 计算包围盒（有些 Xform 没 mesh 会失败，忽略）
    try:
        bbox = UsdGeom.Boundable(prim).ComputeWorldBound(Usd.TimeCode.Default(), purpose1="default")
        aligned = bbox.ComputeAlignedBox()
        size = aligned.GetMax() - aligned.GetMin()
        # 地板启发式：Z 方向极薄 + X/Y 很大
        if size[2] < 0.02 and (size[0] > 2.0 or size[1] > 2.0):
            return 'static'
    except Exception:
        pass
    return 'dynamic'  # 默认倾向于给碰撞（可根据需要改成 skip）

def ensure_scale(stage, root_path, scale):
    if scale is None:
        return
    prim = stage.GetPrimAtPath(root_path)
    if not prim:
        return
    attr = prim.GetAttribute("xformOp:scale")
    if not attr:
        # 创建一个 scale op（简单写法：追加一个 scale 变换）
        xform = UsdGeom.Xformable(prim)
        xform.AddScaleOp().Set(Gf.Vec3f(scale, scale, scale))
    else:
        attr.Set(Gf.Vec3f(scale, scale, scale))

def add_collider_and_optional_rigid(prim, dynamic: bool, max_hulls: int, density: float | None):
    # utils.setRigidBody(prim, approximation, kinematic) 说明：
    #   - prim: 要设置的 Prim
    #   - approximation: "convexDecomposition" | "convexHull" | ...
    #   - kinematic: bool (True -> 运动学体，不受力)
    if dynamic:
        physx_utils.setRigidBody(prim, "convexDecomposition", False)  # 动态刚体，使用凸分解
    else:
        # 对静态物体：仍需一个 collider，但不需要动力学求解，可用 kinematic 或 approximation=convexHull
        physx_utils.setRigidBody(prim, "convexHull", True)  # True -> kinematic（不会被重力影响）
    # 调整凸分解参数（只对存在子 Mesh 的情况生效）
    for child in prim.GetChildren():
        attr = child.GetAttribute("physxConvexDecompositionCollision:maxConvexHulls")
        if attr:
            attr.Set(max_hulls)
    if density and dynamic:
        physicsUtils.add_density(prim.GetStage(), prim.GetPath(), density)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--input', required=True)
    ap.add_argument('--output', required=True)
    ap.add_argument('--scale', type=float, default=None, help='统一缩放（例如 0.001：mm→m）')
    ap.add_argument('--density', type=float, default=100.0)
    ap.add_argument('--max-hulls', type=int, default=64)
    ap.add_argument('--root', default='/World', help='场景根路径')
    ap.add_argument('--filter', default=None, help='仅处理路径包含此子串的 Prim')
    ap.add_argument('--dry-run', action='store_true')
    args = ap.parse_args()

    stage = Usd.Stage.Open(args.input)
    ensure_scale(stage, args.root, args.scale)

    stats = {'dynamic':0, 'static':0, 'skip':0}
    processed = []
    for prim in stage.Traverse():
        if args.filter and args.filter not in prim.GetPath().pathString:
            continue
        category = classify(prim)
        if category == 'skip':
            stats['skip'] += 1
            continue
        dynamic = (category == 'dynamic')
        processed.append((prim.GetPath().pathString, category))
        if not args.dry_run:
            add_collider_and_optional_rigid(prim, dynamic, args.max_hulls, args.density)
        stats[category] += 1

    if not args.dry_run:
        stage.GetRootLayer().Export(args.output)
    print('Summary:', stats)
    for p, c in processed[:30]:
        print('[Processed]', c, p)
    if len(processed) > 30:
        print(f'... ({len(processed)-30} more)')

if __name__ == '__main__':
    main()
    simulation_app.close()
```

## 与原脚本差异说明
| 项 | 原示例 | 新脚本 | 价值 |
|----|--------|--------|------|
| 缩放 | 直接写死 scale | 参数化 `--scale` | 通用 | 
| 分类 | 只处理单一固定路径 | 遍历 + 名称 / 尺寸启发式 | 批量自动化 |
| 刚体设置 | 单个 `utils.setRigidBody` | 动态/静态分流 + kinematic | 更细控制 |
| 凸分解参数 | 手工改一个 child | 批量遍历 children | 适配多结构 |
| 干跑模式 | 无 | `--dry-run` | 安全预览 |
| 统计输出 | 无 | 动态/静态/跳过计数 | 可评估处理质量 |

## 常见问题 (FAQ)
| 问题 | 说明 | 解决 |
|------|------|------|
| 某些 Mesh 烹饪失败 | 退化/超薄几何 | 先运行几何扫描脚本清理 |
| 性能下降 | 大量 convexDecomposition | 改为 convexHull 或手工低模 collider |
| 家具被当成 static | 名称未匹配 | 扩展 DYNAMIC_NAME_PATTERN | 
| 地板被识别为 dynamic | 缩放或命名异常 | 校正 scale / 添加名称标记 |
| 需要区分可破坏对象 | 附加自定义属性标签 | 后续再二次处理 |

## 验证步骤
1. 运行 `--dry-run` 观察分类统计是否合理。
2. 真正导出后在 Isaac Sim 打开新 USD：
   - 检查 `/World` 下对象是否具备 `Rigid Body` / Collider 属性。
   - 随机给一个动态家具添加初速度测试是否受重力影响。
3. 碰撞可视化：打开 Physics Debug Draw，确认包围体与模型合理贴合。

## 进一步改进建议
- 将分类规则外置 JSON（可配置名称列表 / 正则）。
- 加入对退化 Mesh 的自动过滤（如果面数为 0 则跳过）。
- 失败重试：首次 convexDecomposition 失败 → 回退 convexHull。
- 记录日志 JSON（哪些 Prim 改动了哪些属性）。

> 总结：通过遍历 + 启发式分类 + 标准 API 调用，可以在无需手工操作 GUI 的情况下，将一个“纯视觉 USD 场景”快速转换为具有基础物理交互能力的仿真场景，为后续任务（抓取、导航、交互）大幅节省准备时间。
