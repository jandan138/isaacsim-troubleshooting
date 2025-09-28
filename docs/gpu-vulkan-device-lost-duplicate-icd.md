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

## 可能触发崩溃的链路（超详细 / 新手友好讲解）
下面把“为什么两个重复的 `nvidia_icd.json` 最终会演化成 `ERROR_DEVICE_LOST`” 拆成四个阶段。可以把整个过程想象成：
“系统在开场点兵（枚举设备）时给了两份几乎一样但顺序细节不同的花名册 → 引擎记混了人 → 后面发装备（上传资源）时给错人 → 任务执行中有人不认这份指令 → 全队解散（device lost）”。

### 阶段 1：启动枚举（装载花名册）
Vulkan Loader 启动时会扫描两个目录：`/etc/vulkan/icd.d` 与 `/usr/share/vulkan/icd.d`。
它发现两份同名/同作用的 `nvidia_icd.json`，于是：
1. 可能把同一个物理 GPU 记录为两个候选“驱动提供者”。
2. 或者加载顺序导致“先载入 A 版本，再载入 B 版本”覆盖或追加能力列表。
3. 某些 loader 版本会给警告然后继续（并没有立刻终止），于是隐患被保留。

新手要点：这一步不会崩，它只是“埋了雷”。

### 阶段 2：创建逻辑设备与队列（挑选谁来干活）
Isaac Sim 的渲染/RTX 插件需要：图形队列 (graphics queue)、计算队列 (compute queue)、传输队列 (transfer queue) 等。它会查询“这个设备支持哪些扩展 / 队列族 (queue family)”。
如果前一步的重复描述导致：
* 队列家族索引 (family index) 与真实硬件对应关系被错位；
* 扩展能力表混合（比如一个 JSON 声称支持某扩展，另一个没有；最终状态不一致），
那么 Isaac 可能“以为”拿到一个支持特定功能的队列，但其实底层指向的是另一套句柄。早期不会马上暴露，只是留下“错配的句柄引用”。

类比：你以为排队窗口 2 可以办身份证（扩展），结果实际窗口 2 只办社保；暂时没人测试到，于是继续。

### 阶段 3：大批量资源上传（把真实数据放进显存）
运行一段时间后，场景加载 / 贴图 / 几何 / RTX 加速结构构建，会调用大量 `vk*` 函数：
* 创建 / 更新缓冲区 (buffers)
* 贴图上传 (image copy)
* 构建 BLAS/TLAS（若开启 RTX）
这些操作需要 fence / queue 同步：
* Isaac 申请了 fence A，等待它表示“前一批拷贝完成”；
* 但如果之前“队列索引混乱”，实际提交去的是“另一个队列”或状态不一致；
* 等待操作 `vkWaitForFences` 发现句柄无效或内部状态机异常 → 驱动触发断言，将整个设备标记为 LOST。

出现 `Failed copyToDevice` 就是上传指令链某个环节被打断。（驱动为了自保会把设备标记成 lost，防止继续提交造成更严重的不一致）。

### 阶段 4：统一对外报错（顶层看到一个笼统原因）
当底层返回 `VK_ERROR_DEVICE_LOST`，此时：
* 框架已经无法“修复”旧状态（GPU 上命令缓冲区可能半写）。
* 上层不再尝试细分具体是哪条指令导致，仅打印“device lost, out of memory, or unexpected bug”。
因此你看到的是一个“含糊三选一”提示，但真实根因是前面“设备初始化匹配/描述冲突”埋下的链式问题。

### 为什么不是所有机器都会炸？
| 情况 | 是否容易触发 | 解释 |
|------|---------------|------|
| Loader 版本更宽容 | 不容易 | 直接忽略重复或合并后保持一致结构 |
| 场景资源少 | 不容易 | 没有大量异步拷贝/构建，较少同步压力 |
| 使用少量渲染特性（不开 RTX / DLSS） | 不容易 | 触发路径减少 |
| 双份 JSON 内容完全一致且顺序稳定 | 低概率 | 即使重复也未混乱队列与扩展表 |

### 极简归纳
重复 ICD → 设备“被描述两次” → 队列/扩展集合潜在错配 → 大量 GPU 任务同步阶段暴露矛盾 → 驱动自杀保护 → `ERROR_DEVICE_LOST`。

### 给新手的“我到底要做什么”
1. 先确认是不是重复 ICD：两个目录里都有 `nvidia_icd.json`。
2. 备份其中一份（改名）。
3. 重启再看启动日志是否还出现 Multiple ICD 警告。
4. 再做同样的场景/资源加载操作，看是否还会触发。
5. 若仍触发，再回到“其他可能干扰因素”表逐项排除。

> 记忆法：先消除“多身份证”→ 再观察“排队窗口是否还乱” → 最后才考虑“是不是身体（硬件）本身有病（Xid / 过热）”。


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
