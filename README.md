# Isaac Sim Troubleshooting

记录在使用 NVIDIA Isaac Sim 过程中的常见问题、原因分析与解决方案，旨在帮助自己与他人快速定位并解决问题，减少重复踩坑时间。

## 仓库结构
```
isaacsim-troubleshooting/
├── README.md
├── docs/                  # 具体问题与解决方案文档（每个问题一个 md）
│   ├── installation-error-1.md
│   ├── startup-crash-prestartup.md
│   ├── material-missing.md
│   ├── multi-gpu-issue.md
│   ├── problem-template.md
│   └── ...
└── images/                # 截图与配图资源
    ├── installation-error-1.png
    ├── startup-crash-prestartup.png
    └── ...
```

## 使用说明
- 所有问题文档位于 `docs/` 目录。
- 每个问题一个独立 Markdown 文件，命名使用短横线（kebab-case），并尽量表达关键信息。
- 文档中引用图片使用相对路径，例如：`![示例](../images/installation-error-1.png)`。
- 新增问题时可以复制模板：`docs/problem-template.md`。

## 快速导航
| 分类 | 问题 | 链接 |
|------|------|------|
| 安装 | 安装报错：某组件缺失 | [installation-error-1](docs/installation-error-1.md) |
| 启动 | 启动阶段 PreStartup 崩溃 | [startup-crash-prestartup](docs/startup-crash-prestartup.md) |
| 资源 | 材质缺失或粉色显示 | [material-missing](docs/material-missing.md) |
| 多 GPU | 多 GPU 设备识别/调度异常 | [multi-gpu-issue](docs/multi-gpu-issue.md) |
| 性能 | GUI 直接打开 USD 仿真卡顿 | [gui-simulation-lag-direct-usd](docs/gui-simulation-lag-direct-usd.md) |

> 后续会按需增加分类（网络、ROS、路径规划、可视化、性能优化等）。

## 贡献方式
欢迎 PR / Issue：
1. 复现步骤是否明确。
2. 原因分析是否有依据（官方文档、日志、源码片段）。
3. 解决步骤可操作、最好包含验证方式。

## 许可证
本仓库内容默认采用 MIT License（可根据需要修改）。截图若包含第三方受版权保护内容，请在上下文中注明来源。
