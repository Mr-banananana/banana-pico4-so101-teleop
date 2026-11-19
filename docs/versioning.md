# 版本管理

## 版本号

项目使用语义化版本号 `MAJOR.MINOR.PATCH`，唯一版本源为 `mujoco_pick_place/__init__.py`。服务状态、诊断包和启动器都从这里读取版本，不再维护单独的构建字符串。

- `PATCH`：不改变使用方式的缺陷修复。
- `MINOR`：向后兼容的新功能。
- `MAJOR`：需要使用者调整配置或操作方式的不兼容变更。

## 发布步骤

1. 在 `CHANGELOG.md` 记录本次新增、修改和修复。
2. 运行 `python -m unittest discover -v`，并检查 `git diff --check`。
3. 更新 `mujoco_pick_place/__init__.py` 的版本号，提交代码。
4. 为提交打 `vMAJOR.MINOR.PATCH` 标签并推送分支和标签。
5. 在 GitHub 为该标签创建 Release，发布说明从对应的更新日志条目整理。

本机标定文件、服务会话、trace 和运行产物保存在 `artifacts/` 或本机配置中，不提交到仓库。
