---
title: 安装报错：某组件缺失
category: 安装
order: 10
---

# 安装报错：某组件缺失

## 问题现象
执行安装或首次启动 Isaac Sim 的过程中，终端/日志出现类似：
```
ImportError: libpython3.10.so.1.0: cannot open shared object file: No such file or directory
```
或：
```
Failed to load extension omni.kit.window.viewport
```
![安装报错示例](../images/installation-error-1.png)

## 原因分析
- 系统缺少对应的动态库或运行时依赖（如 Python 运行时 / NVIDIA 驱动组件）。
- Isaac Sim 解压或同步不完整（网络中断导致部分文件缺失）。
- 与本地已有 Conda / Python 环境变量冲突（如 `PYTHONPATH` 覆盖）。

## 解决方法
1. 确认系统依赖：
   - Ubuntu: 运行 `apt list --installed | grep -E "vulkan|nvidia"` 查看驱动和 Vulkan 组件。
2. 重新执行官方安装脚本，或通过 Omniverse Launcher 重新 Repair。
3. 清理环境变量冲突：
   - 暂时移除自定义 `PYTHONPATH`，或在独立 Shell 中执行。
4. 校验文件完整性：
   - 对比 `kit/exts/` 目录是否存在报错中提到的扩展。
5. 再次启动并观察日志关键行：
```
./isaac-sim.sh --no-window
```
若日志展示所有必需 extension 均加载成功，则问题解决。
