---
title: 启动崩溃：PreStartup 阶段退出
category: 启动
order: 10
---

# 启动崩溃：PreStartup 阶段退出

## 问题现象
启动 Isaac Sim（GUI 或 `--no-window`）时，日志在 `PreStartup` 阶段终止：
```
[Info] [omni.app] Startup sequence: PreStartup
[Error] [omni.ext.plugin] Failed to startup extension omni.x.y.z
[Error] [carb] RuntimeError: xxx.so: undefined symbol: _ZN...
```
并直接返回桌面或进程退出（退出码 139 / 1）。

![PreStartup 崩溃示例](../images/startup-crash-prestartup.png)

## 原因分析
- 某个关键 extension 初始化失败（依赖未满足 / 版本不匹配）。
- GPU 驱动版本过低，不支持当前 Isaac Sim 要求的 CUDA / Vulkan 特性。
- 头一次启动缓存损坏（`~/.nvidia-omniverse` 下缓存结构异常）。
- 环境变量导致动态库解析到错误路径（例如 LD_LIBRARY_PATH 混入旧版本库）。

## 解决方法
1. 检查 GPU 驱动：
```
nvidia-smi
```
确认驱动版本满足官方 Release Notes 要求。
2. 以最小加载启动：
```
./isaac-sim.sh --ext-folder "" --no-window
```
若能进入则逐步恢复自定义 extensions。
3. 临时重命名缓存目录并重试：
```
mv ~/.nvidia-omniverse ~/.nvidia-omniverse.bak
```
4. 检查失败的 extension：搜索日志 `Failed to startup extension`，在 `kit/exts/EXT_NAME/config/extension.toml` 查看依赖。
5. 若符号缺失（undefined symbol），利用 `ldd` 检查对应 `.so`：
```
ldd path/to/libxxx.so | grep not
```
6. 重装 / Repair Isaac Sim，或同步最新 patch。

## 验证与回归
- 再次启动时应看到完整启动阶段：`PreStartup -> Startup -> PostStartup -> Running`。
- 不再出现崩溃且可正常加载场景。
