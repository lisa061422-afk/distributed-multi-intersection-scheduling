# ADMM Hyperparameters — MAIN_Parallel_Compute.m

| 参数 | 变量名 | 默认值 | 位置（行） | 说明 |
|------|--------|--------|-----------|------|
| ADMM 惩罚系数 1 | `rho1` | `1` | 108 | 控制 x/y 对偶残差的惩罚强度 |
| ADMM 惩罚系数 2 | `rho2` | `1` | 108 | 控制 alpha/gamma 对偶残差的惩罚强度 |
| 目标权重 | `weight` | `1.5` | 108 | 路口等待 vs 道路延迟的加权系数 |
| 最大迭代次数 | `max_iter` | `500` | 108 | ADMM 迭代上限 |
| 原始残差容忍 | `tol_r` | `1e-2` | 109 | 收敛判据：原始残差 r < tol_r |
| 对偶残差容忍 | `tol_s` | `1e-2` | 109 | 收敛判据：对偶残差 s < tol_s |
| 随机初始化幅度 | `randInitScale` | `2` | 117 | >0：每辆车随机延迟 [0, scale] 秒；0：最早时间初始化 |
| 多起点次数 | `numStarts` | `1` | 118 | 1 = 单次运行；>1 = 多起点取最优 cost |
| 优先级覆盖 | `const.priority_n` | `0` | 328 | 0 = 无优先级约束（正常运行） |
| 树剪枝开关 | `const.use_pruning` | `true` | 329 | true = 剪去劣解节点（大 N 更快） |
| 弱规则锁开关 | `const.use_weak_rule` | `true` | 330 | 分布式端口必须为 true |
| 单 agent 超时 | `const.timeout_int_s` | `30` | 331 | 每个 agent 树搜索超时（秒） |
| 时间界剪枝 | `const.useTBound` | `true` | 332 | 剪去超出最坏顺序 deadline 的节点 |
| 求解器选择 | `const.use_quadprog` | `true` | 333 | true = quadprog（快）；false = YALMIP+Gurobi（原始） |
| 自适应 ρ 开关 | `const.use_adaptive_rho` | `true` | 334 | Boyd 2011：r/s>10 → ρ×2；s/r>10 → ρ/2 |
