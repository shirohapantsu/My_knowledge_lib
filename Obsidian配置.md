---
tags:
- 工具
created: 2026-09-25
up: "[[工具 MOC]]"
status:
---

> 📌 **知识路径**：

---

## 1. Git 同步配置

**电脑Git同步配置：**
1. [[安装与初始化Git]]
2. **安装插件：**在Obsidian中打开"设置/第三方插件"，关闭安全模式，安装插件"Git"
3. **初始化仓库：**输入快捷键"`Cmd + P`"，搜索栏输入 `Git: Initialize a new repo`，进行初始化
4. **打开控制面板：**输入快捷键"`Cmd + P`"，搜索栏输入`Git: Open source control view`，打开Git控制面板
5. **配置关联Github：**输入快捷键"`Cmd + P`"，搜索栏输入 `Git: Edit remotes`，Remote name选`origin`，url格式：`https://github.com/你的用户名/你的仓库名.git`
6. **初始化本地Branch：**在根目录下新建`main`分支
```bash
git init
git branch -M main
git add .
git commit -m "Initial commit"
```
7. **配置远程Branch：**输入快捷键"`Cmd + P`"，搜索栏输入 `Git: Set upstream branch`，选择：`origin/main`，完成主分支设置
8. **配置自动同步：**在插件设置中设置`Auto commit-and-sync interval`和`Auto pull interval`

**手机Git同步配置：**

---

## 2. Obsidian内部配置

1. 配置模板 
2. 配置源码切换快捷键和默认编辑模式