---
title: 多 GPU 设备识别 / 调度异常
category: 多 GPU
order: 10
---

# 多 GPU 设备识别 / 调度异常

## 问题现象
系统存在多块 GPU，但 Isaac Sim 仅使用一块，或使用的并非期望 GPU；`nvidia-smi` 观察到显存占用不均衡。日志可能缺少对所有 GPU 的枚举输出。

## 原因分析
- 驱动未正确安装（某些 GPU 未被驱动识别）。
- 环境变量 `CUDA_VISIBLE_DEVICES` 限制了可见 GPU。
- Isaac Sim 当前版本对多 GPU 支持有限（部分功能仍单 GPU）。
- 混合显卡（集显 + 独显）导致优先级选择异常。

## 解决方法
1. 核对驱动识别：
```
nvidia-smi -L
```
2. 确认未被隐藏：
```
echo $CUDA_VISIBLE_DEVICES
```
为空或包含所有需要索引。
3. 使用单 GPU 绑定测试：
```
CUDA_VISIBLE_DEVICES=0 ./isaac-sim.sh --no-window
```
4. 查看 Isaac Sim 日志内 GPU 分配：搜索 `GPU` / `device` 关键字。
5. 若使用 Docker：
```
--gpus all
```
并确认容器内 `nvidia-container-cli` 输出。
6. 关注官方 Release Notes 中对多 GPU 的限制说明（某些渲染/仿真模块暂不做多 GPU 并行）。

## 验证与回归
- 分配策略符合预期，关键模块（渲染 / 物理）运行时显存占用稳定。
- 不再出现 GPU 切换导致的崩溃或性能异常。
