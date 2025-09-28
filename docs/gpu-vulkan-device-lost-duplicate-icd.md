---
title: GPU Device Lost / Vulkan ERROR_DEVICE_LOST 与重复 ICD JSON 冲突
category: GPU
order: 15
---

# GPU 运行中突然崩溃：`VkResult: ERROR_DEVICE_LOST` 且启动时提示多重 ICD

## 问题现象
单机（示例：编号 734）运行 Isaac Sim 一段时间后突然退出，日志末尾出现：
```
[Error] [carb.graphics-vulkan.plugin] VkResult: ERROR_DEVICE_LOST
[Error] [carb.graphics-vulkan.plugin] vkWaitForFences failed for command queue.
[Error] [omni.kit.renderer.plugin] Failed copyToDevice
[Error] [carb.graphics-vulkan.plugin] submitToQueueCommon failed.
[Error] [gpu.foundation.plugin] A GPU crash occurred. Exiting the application...
Reasons for the failure: a device lost, out of memory, or an unexpected bug.
```
同时在**启动早期**曾输出（截取）：
```
[Error] [gpu.foundation.plugin] Multiple Installable Client Drivers (ICDs) are found for the same GPU ... leads to instability or crash.
To verify the proper NVIDIA driver installation, only one of the following icd.d folders should contain nvidia_icd.json file:
 /etc/vulkan/icd.d
 /usr/share/vulkan/icd.d
```
检查系统：
```
ls /etc/vulkan/icd.d/        # 存在 nvidia_icd.json
ls /usr/share/vulkan/icd.d/  # 也存在 nvidia_icd.json 以及 intel / radeon / virtio 等
```
→ 说明同一 NVIDIA GPU 被多个 ICD JSON 暴露，Vulkan 层可能重复枚举 / 混淆，导致后续渲染管线不稳定最终触发 `ERROR_DEVICE_LOST`。

实际处理：将其中一个目录（一般是 `/etc/vulkan/icd.d/`）下的 `nvidia_icd.json` 临时改名（如 `nvidia_icd.json-bk`），重启后崩溃未再复现。

## 背景知识：什么是 Vulkan ICD JSON？
Vulkan 使用“Installable Client Driver (ICD)”机制：
| 文件 | 作用 |
|------|------|
| `nvidia_icd.json` | 指向 NVIDIA Vulkan 驱动共享库（`libnvidia-vulkan.so` 等）的描述文件 |
| `intel_icd.*.json` | 指向 Intel (集显) Vulkan 驱动 |
| `radeon_icd.*.json` | 指向 AMD 驱动 |
驱动加载器会扫描标准路径（通常 `/etc/vulkan/icd.d` 与 `/usr/share/vulkan/icd.d`）。**同一 GPU 若被多个重复 JSON 文件指向（或发行版+厂商安装各放一份），可能令上层框架误判或在选择物理设备时进入未测试分支**。

## 可能触发崩溃的链路
1. 启动阶段多 ICD → 设备枚举顺序/索引不稳定。
2. Isaac Sim / 依赖插件选择了错误的 queue family 或混合了扩展能力表。
3. 在后续大批量上传资源（`copyToDevice`）时触发驱动内部断言 → `VkResult: ERROR_DEVICE_LOST`。
4. 框架捕获不到更细节的根因，统一报告 device lost。

并非所有系统都会这样崩，因为：
- 某些版本的 Vulkan loader 更宽容（忽略重复）。
- GPU 负载较小时不会触发后续访问到问题路径。

## 排查流程 Checklist
| 步骤 | 命令 / 操作 | 期望 / 处理 |
|------|-------------|-------------|
| 1. 列出 ICD | `ls /etc/vulkan/icd.d/`; `ls /usr/share/vulkan/icd.d/` | 确认是否双目录都有 `nvidia_icd.json` |
| 2. 查看文件差异 | `diff /etc/vulkan/icd.d/nvidia_icd.json /usr/share/vulkan/icd.d/nvidia_icd.json` | 通常内容相同或极少差异 |
| 3. 备份其中一份 | `sudo mv /etc/vulkan/icd.d/nvidia_icd.json /etc/vulkan/icd.d/nvidia_icd.json-bk` | 保留单一实例 |
| 4. 清理缓存 (可选) | 删除 `~/.cache/ov/` 中 renderer 临时 | 让下一次启动走干净路径 |
| 5. 重启 Isaac Sim | 观察启动日志 | 不再出现 Multiple ICD 警告 |
| 6. 压力复现 | 加载同场景 / 跑脚本 | 不再随机 device lost |

## 其他可能的干扰因素（确认后再排除）
| 因素 | 说明 | 区分方式 |
|------|------|----------|
| 真显存 OOM | 大纹理 / RTX GI / 高分辨率多视口 | 监控 `nvidia-smi`，崩溃前显存未满则排除 |
| GPU 频繁超频/温度 | 硬件稳定性问题 | `dmesg` 是否出现 Xid 错误；温度是否临界 |
| 驱动版本不匹配 | Isaac 推荐版本与系统不一致 | `nvidia-smi` 显示 driver vs release notes 建议版本 |
| 混合显卡 PRIME | 集显 + 独显冲突 | 禁用集显或在 BIOS / 驱动层切换独显模式 |
| Wayland/窗口系统 | 某些组合触发 Vulkan 层兼容 bug | 切换到 X11 session 复测 |

## 自动检测脚本示例
```bash
#!/usr/bin/env bash
set -e
N1=/etc/vulkan/icd.d/nvidia_icd.json
N2=/usr/share/vulkan/icd.d/nvidia_icd.json
count=0
for f in "$N1" "$N2"; do
  [ -f "$f" ] && count=$((count+1))
done
if [ $count -gt 1 ]; then
  echo "[WARN] Detected duplicate NVIDIA ICD json ($count)." >&2
  echo "Suggest: backup one: sudo mv $N1 ${N1}-bk" >&2
  exit 2
else
  echo "[OK] No duplicate NVIDIA ICD present."
fi
```

## 安全处置建议
1. 只保留一份 **有效且最新** 的 `nvidia_icd.json`（通常发行版包管理放 `/usr/share/vulkan/icd.d/`；/etc 下的可能来自手动安装包）。
2. 不要直接删除，先改名备份：`mv nvidia_icd.json nvidia_icd.json-bk`，确认稳定后再清理。
3. 若未来驱动更新重新生成双份，重复上述检查。

## 回归验证
| 项 | 通过标准 |
|----|-----------|
| 启动日志 | 不再打印 Multiple Installable Client Drivers 警告 |
| 运行时 | 长时间加载 / 渲染不再触发 `ERROR_DEVICE_LOST` |
| GPU 监控 | 显存占用曲线平稳，无意外骤降 |
| 重命名回滚 | 把备份文件改回原名 → 警告重新出现（验证因果） |

## FAQ
**Q: 保留哪一份更好?** 一般保留发行版包管理安装位置（`/usr/share/...`），移走 `/etc/...` 的自定义副本。若有专门文档说明相反策略则遵循团队标准。

**Q: 会影响非 Isaac 的 Vulkan 程序吗?** 只要保留一份有效 JSON 不会；多份反而可能影响其它高负载渲染应用。

**Q: Windows 也有这个问题吗?** Windows 采用不同驱动分发机制，不使用同样的多目录 ICD JSON 模式，一般不会发生该类冲突。

**Q: 还是会 device lost?** 继续检视 `dmesg` 中的 Xid（硬件/驱动级错误），以及是否有超频或不稳定电源供应。

> 总结：本案例核心是 **重复 Vulkan ICD 描述文件导致的驱动层混乱**。清理为单一 NVIDIA ICD 后即可稳定。Device Lost 并非总是显存不足，也可能是加载/初始化阶段设备选择与后续命令缓冲被错误驱动实例处理引发的崩溃。
