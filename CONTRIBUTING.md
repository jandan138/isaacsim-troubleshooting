# 贡献指南 (Contributing)

感谢你愿意为本仓库贡献内容！请在提交 Issue 或 Pull Request 前阅读以下规范。

## 内容范围
聚焦 Isaac Sim / Omniverse 使用相关的：
- 安装 & 环境依赖
- 启动 & 扩展加载
- 资源/材质/渲染
- 物理与性能调优
- GPU / 多 GPU / 驱动问题
- 场景/资产兼容性

## 新增问题文档流程
1. 复制 `docs/problem-template.md` → 重命名（kebab-case，例如：`gpu-memory-fragmentation.md`）。
2. 填写：问题现象 / 原因分析 / 解决方法 / 验证与回归 / 参考资料。
3. 添加截图到 `images/`，文件名与文档 slug 对应（可追加后缀区分步骤）。
4. 在 YAML Front Matter 中补充：
   ```yaml
   ---
   title: GPU 内存碎片导致加载缓慢
   category: 性能
   order: 20
   ---
   ```
5. 运行：
   ```bash
   python scripts/generate_nav.py
   ```
6. 提交 PR。

## 分类建议 (category)
安装, 启动, 资源, 性能, 多 GPU, 网络, ROS, 路径规划, 可视化, 调试, 兼容性, 其他。

## 写作建议
- 标题精准：避免“某问题解决记录”此类模糊描述。
- 问题现象：包含环境关键信息（Isaac Sim 版本 / OS / GPU / 驱动 / CUDA）。
- 原因分析：给出推断依据（日志关键行 / 官方文档链接 / 实验对比）。
- 解决方法：分步骤，可复制命令，必要时给出替代方案。
- 验证与回归：说明如何确认彻底解决以及防止回归。

## 代码 / 日志片段
- 保留必要上下文，避免贴整屏无关输出。
- 可使用：
  ```
  [Error] [omni.ext.plugin] Failed to startup extension omni.x.y.z
  ```

## 命名规范
- 文件：全部小写，单词用短横线：`startup-crash-prestartup.md`
- 图片：`同名文档slug[-步骤/场景].png`

## 英文翻译
若提供英文版，放入 `docs/en/` 并保持 slug 对应，必要时可在 front matter 添加 `lang: en`。

## 提交规范
Commit message 使用动词前缀：
- docs: add xxx
- fix: correct xxx
- chore: update nav script

## 审核标准
- 是否复现或可复现。
- 是否重复（已有条目是否涵盖）。
- 是否存在未注明的风险（例如：修改系统库 / 删除关键缓存）。

## License
提交内容默认按仓库 MIT 协议发布。若引用外部内容请注明来源链接。

欢迎改进本指南！
