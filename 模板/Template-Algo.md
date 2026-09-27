---
tags:
  - algorithm
  - algorithm/<分类，如：dp/graph/tree/sliding-window>
difficulty: Medium # Easy / Medium / Hard
platform: LeetCode # LeetCode / Codeforces / 洛谷 / 模板
problem_id: 
source_url: 
status: 🔄待复习 # ⏳做不出 / 🔄待复习 / ✅已掌握
review_count: 0
created:  {{date}}
---

# {{title}}

> [!quote] 题目链接 / 来源
> - 题目地址：[Link]({{source_url}})
> - 相关模式：[[ ]]

---

## 1. 题意提炼与约束 (Problem & Constraints)
* **核心输入输出**：
* **数据范围与关键约束**：
  * $N \le 10^5 \implies$ 复杂度需控制在 $O(N)$ 或 $O(N \log N)$
  * 存在负数 / 空输入等边界情况？

---

## 2. 核心思路与解法演进 (Intuition & Approach)

> [!tip] 核心破局点（Insight）
> 为什么能想到用这个方法？贪心/状态转移/单调性的依据是什么？

### 方案分析
1. **暴力解法（Brute Force）**：
   * 思路与瓶颈：
2. **最优解法（Optimal）**：
   * **状态定义 / 不变量**：
   * **转移方程 / 贪心策略**：

---

## 3. 代码实现 (Implementation)

> [!example] Python3 / C++ 实现
```python
class Solution:
    def solve(self) -> int:
        # 1. 边界处理
        
        # 2. 核心逻辑
        
        # 3. 返回结果
        pass
```

---

## 4. 复杂度分析 (Complexity)
* **时间复杂度**：$O(N)$，解释：
* **空间复杂度**：$O(1)$，解释：

---

## 5. 易错点与边界条件 (Pitfalls & Edge Cases)
> [!danger] 避坑指南
> - [ ] 数组越界（索引 $0$ 或 $n-1$）
> - [ ] 整型溢出（如使用 long long）
> - [ ] 空指针 / 空输入处理

---

## 6. 变体与相似题目 (Similar Problems)
* [[相似题 1]] - 相同思路，不同应用场景
* [[相似题 2]] - 进阶版（带附加条件）