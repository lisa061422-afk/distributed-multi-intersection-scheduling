# WeakRule ON 算法核心修改总结

## Mod 1：Pairwise Priority Lock（pair_lock）

**问题：** 两系统反复竞争同一 resource，每次都生成新分支 → 搜索树爆炸 / cycling。

**修改：** 引入 N×N 矩阵 `pair_lock(i,j) = winner`，记录每对系统的竞争结果。

**Auto-win 条件：** 若 candidate c 满足：
1. `pair_lock(c, o) = c` 对所有竞争者 o 成立
2. c 正在执行中（`ra > 0`）

则 c 直接获胜，跳过分支。

**完备性保证：** 第一次竞争时已枚举所有优先级组合（每对生成两个 branch）；之后 lock 只防止同一 path 上重复分支，不影响兄弟 branch 的独立展开。新系统介入时 pair_lock=0 → 自动触发新分支。

**关键设计：** lock 永不清零（清零是旧版 cycling bug 的根源）。

---

## Mod 2：x 与 V_temp 共同决策（space_variants）

**问题：** 旧方法中 reset 后的 sub-task 重排（x）单向依赖 V_temp 当前占用者，无法找到需要调整 V_temp 的更优解。

**修改：** 引入 `space_variants`，对"被 reset 的系统 n 重新需要 space m"生成多个 branch：

| pair_lock(n, np) | 生成 branch |
|---|---|
| 0（未竞争过） | wait + displace 两个 |
| = np（np 赢过） | 只有 wait |
| = n（n 赢过） | 只有 displace |

**Displace branch：** np 被踢出 V_temp 并整体 reset，n 获得 space m 的优先权。

**约束：** 只修改 V_temp 里其他系统的 pending 请求，不改动 traverse_columns 已 commit 的 V_valid 分配结果。

---

## Mod 3：reset_since 标记失效记录

**问题：** 系统 n 被 reset 后，其历史 V 占用记录应失效，但 V 由父节点共享不能直接修改。

**修改：** 节点新增向量 `reset_since[n]`：
- n 在时刻 tw 被 reset → `reset_since[n] = tw`
- `x{n}{k} = {}` 清空当前 vehicle 的预约记录

**使用：** `check_resc_occupation` 检查 V 时忽略 n 在 `reset_since[n]` 之前的所有记录。

**最终 schedule 重建：**
- `gamma[n]`：sub-task 完成时写入的实际占用区间（主体）
- `x{n}{k}`：reset 过程生成的预约区间 `{t_start, t_end, sub, space}`（补充未完成部分）
- 两者合并，过滤 reset_since 后得到完整调度时间轴

---

## Mod 4：T_bound 剪枝

**问题：** 搜索中存在大量明显劣于串行基准的无效路径。

**修改：** 计算串行最坏情况上界：

$$T_{\text{bound}} = \max_n \tilde{\alpha}_n + \sum_{n=1}^{N}\sum_{s=1}^{S} C_{s,n}$$

其中 $\tilde{\alpha}_n = d1_n + \frac{R/2 - W/2}{v_{\max}}$ 为系统 n 最早到达 merge zone 的时刻。

**剪枝规则：** OPEN 中 `tw > T_bound` 的节点直接移除。

**含义：** 最晚到达的系统抵达后，把所有 sub-task 串行排队——任何合理调度都应优于此，超出则无意义。
