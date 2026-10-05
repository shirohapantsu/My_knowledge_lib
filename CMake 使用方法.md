---
tags: 
- 开发工具
created: 2026-10-04
up: "[[ ]]"
status:
---

# CMake 使用方法

CMake 是一个构建系统生成器：写 `CMakeLists.txt` 描述“有哪些源码、生成什么目标、依赖哪些库”，然后 CMake 再生成 Makefile、Ninja 文件、Visual Studio 工程等，最后由真正的编译器完成编译。

**现代 CMake 一切以 target 为核心**

## 1. 常规 CMake 头

```CMake
cmake_minimum_required(VERSION 3.16) # 最小CMake版本需求3.16

project(HelloCMake LANGUAGES CXX） # 项目名为HelloCMake，语言为C++

set(CMAKE_CXX_STANDARD 17) # 使用C++17标准
set(CMAKE_CXX_STANDARD_REQUIRED ON) # 要求必须满足这个标准
set(set(CMAKE_CXX_EXTENSIONS OFF)) # 关闭编译器的语言拓展
```

这部分完成了对编译器CMake的基本配置，基本上所有项目都会以这些内容开头

---

## 2. 引入外部依赖

