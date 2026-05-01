# 仓库 Python 端使用说明

任意 4-way 拓扑 + 随机/手动/交互式输入 + 分布式 C-ADMM 调度 + 可视化。

---

## 0. 一次性 setup

```powershell
# 进入仓库根目录（所有命令都从这里跑，不要进 python_port/）
cd "C:\Users\robin\OneDrive\Documents\Github_file\demo_backup_0414\Distributed_schedule_for_warehouse"

# 安装依赖（只需一次）
pip install numpy scipy matplotlib
```

---

## 1. 三种使用方式

### A. 随机模式（最快出结果）

让程序自动随机生成连通图：

```powershell
python python_port/run_arbitrary_demo.py --mode random --n_int 6 --N 8 --seed 42 --plot
```

参数：
- `--n_int 6` 路口数（默认 6，上限 20）
- `--N 8` 车辆数
- `--seed 42` RNG 种子（同 seed 出同样结果）
- `--plot` 弹出 6 张图

### B. 交互模式（鼠标画图，推荐新手）

```powershell
python python_port/run_arbitrary_demo.py --mode interactive --N 8 --seed 42 --plot
```

会弹出一个 6×6 网格窗口让你点 → 按 Q 收工 → 自动跑 ADMM + 出图。

picker 操作详见下面 **第 2 节**。

### C. 手动模式（用 .py spec 文件）

先写一个 `my_topology.py`：

```python
# my_topology.py
name = 'my_map'
coords = {
    1: (0, 1),     # 路口 1 在网格 (col, row) = (0, 1)
    2: (1, 1),
    3: (1, 0),
    4: (0, 0),
}
ports = [
    (1, 'N'), (1, 'W'),    # 路口 1 北侧、西侧各一个端口
    (2, 'N'), (2, 'E'),
    (3, 'E'), (3, 'S'),
    (4, 'S'), (4, 'W'),
]
# 可选：删掉某些自动会连的边
excluded_edges = [{1, 2}]   # I1 和 I2 不连（即使它们正交相邻）
```

然后跑：

```powershell
python python_port/run_arbitrary_demo.py --mode manual --spec my_topology.py --N 8 --plot
```

**已经准备好的示例 spec**：[`example_manual_topo.py`](example_manual_topo.py)（5-int L 形）

---

## 2. 交互式 picker 详解

弹窗是个网格画布。**操作**：

| 鼠标/键盘 | 作用 |
|---|---|
| **左键**点格子 | 加路口（再点同位置 = 删除） |
| **右键**点路口某一侧 | 加端口（再点 = 删除） |
| **中键**（滚轮）点边的中间 | 删除/恢复那条边 |
| **Q** 键 | 提交收工 |
| **R** 键 | 全清重来 |

### 边的规则（重要）

**边不需要你画 —— 是自动生成的**：两个路口在网格上正交相邻（差 1 格、不是对角）→ 自动连边。

```
情况 A：相邻自动连
  ┌──┬──┐         I1 ── I2
  │I1│I2│
  └──┴──┘
  I1 在 (0,0), I2 在 (1,0)

情况 B：隔开就不连
  ┌──┬──┬──┐
  │I1│  │I2│      I1   I2  （没边）
  └──┴──┴──┘
  差 2 格 → 无边
```

不想要某条边？两种方法：
1. **放路口时就放远点**（隔一格放），自动不连
2. **中键删边**，会变灰虚线 + 红 X

### 端口的规则

**不右键也行**：按 Q 提交时，**所有路口的空闲方向自动变成端口**。右键是给你**精细控制**用的——只想要部分方向是端口时才用。

灰色小方块 = 提示"这里能加端口"。

### 颜色含义

| 标记 | 含义 |
|---|---|
| 蓝圆 I1, I2... | 路口 |
| 橙方块 P1, P2... | 已选的端口 |
| 灰小方块 | "这里可以加端口"提示 |
| 黑实线 | 自动连的边 |
| 灰虚线 + 红 X | 你删掉的边 |

### picker 退出后

终端会**打印你画的 spec**：
```python
coords = {1: (0, 0), 2: (1, 0), ...}
ports  = [(1, 'N'), ...]
excluded_edges = [{1, 2}]   # 仅当你删过边
```
复制到 `.py` 文件 → 下次用 `--mode manual --spec foo.py` 复用，不用重画。

---

## 3. 完整参数表

### 拓扑
| flag | 默认 | 含义 |
|---|---|---|
| `--mode` | 必填 | `random` / `manual` / `interactive` |
| `--n_int` | 6 | 路口数（仅 random 模式） |
| `--n_ports` | `2*n_int` | 端口数（仅 random 模式） |
| `--spec` | — | spec 文件路径（仅 manual 模式） |
| `--grid_size` | 6 | picker 网格大小（仅 interactive 模式） |

### 车辆 / 物理
| flag | 默认 | 含义 |
|---|---|---|
| `--N` | 8 | 车辆数 |
| `--seed` | 42 | RNG 种子 |
| `--Dt` | 5.0 | 单条 road 通过时间 (s) |
| `--T_val` | 2.0 | 同入口前后车 headway (s) |
| `--T_ent` | 0.0 | 不同入口的 stagger (s) |

### ADMM
| flag | 默认 | 含义 |
|---|---|---|
| `--max_iter` | 200 | 最大迭代数 |
| `--rho` | 1.0 | 惩罚参数 ρ1=ρ2 |
| `--alpha` | 1.0 | over/under-relaxation（α<1 抑制振荡） |
| `--adaptive_rho` | off | 自动调 ρ |
| `--parallel` | off | 路口 agent 并行 |

### 输出
| flag | 默认 | 含义 |
|---|---|---|
| `--plot` | off | 弹出 matplotlib GUI |
| `--save_plots PATH` | off | 把图存成 PNG 到指定目录（headless） |
| `--quiet` | off | 隐藏每轮 ADMM 日志 |

---

## 4. 输出说明

### 终端
```
Topology "...": n_int=X, n_road=Y, n_ports=Z, n_OD=W
Vehicles (0-indexed agent chains):
  n=0: chain=[...]  alpha_tilde=2.00  ddl=8.36
  ...
Running ADMM...
=== Converged ===
  iterations    = 38 / 200
  delay cost    = 2.2333
  final r, s    = 0.0089, 0.0022
Per-vehicle terminal time vs deadline:
  n=0: t_term=8.356  ddl=8.356  delay=0.000
  ...
```

### 图（`--plot` 或 `--save_plots`）
| 图 | 内容 |
|---|---|
| **fig0_topology** | 你画的 / 生成的地图（路口、路段、端口） |
| **fig1** | ADMM 残差曲线（primal r 和 dual s） |
| **fig2** | Total delay cost 随 iteration 变化 |
| **fig3** | Gantt：每个 merging zone 的车辆占用 |
| **fig4** | Gantt：每条 road + terminal 的车辆占用 |
| **fig5** | 每个路口内部 5 个 conflict space (M1-M5) 的占用 |

**没收敛**时只画 fig0 + fig1 + fig2（schedule 图无意义就不画了）。

---

## 5. 其他脚本

| 脚本 | 用途 |
|---|---|
| [`run_arbitrary_demo.py`](run_arbitrary_demo.py) | 任意拓扑 demo（本说明的主角） |
| [`run_validation.py`](run_validation.py) | 跑原 4-int 数值验证（对齐 MATLAB baseline） |
| [`run_mpc_demo.py`](run_mpc_demo.py) | MPC 滚动优化 demo（4-int 上） |
| [`run_scaling.py`](run_scaling.py) | 多 seed × 多 n_int 扫描 + CSV/PNG 输出 |
| [`run_results.py`](run_results.py) | 跑一次 ADMM 后画 5 张大图（matplotlib GUI） |
| [`pick_topology.py`](pick_topology.py) | 单独跑 picker（不跑 ADMM，只返回拓扑） |
| [`build_manual_config.py`](build_manual_config.py) | 从 hardcoded 4-int 配置重建 validation_config.json |

跑法都是 `python python_port/<script>.py [args]`。

---

## 6. 常见错误 & 调参建议

### ❌ `error: the following arguments are required: --mode`
没传 `--mode` 参数。**不能直接 Run/F5 跑这个脚本**——必须从终端带参数跑：
```powershell
python python_port/run_arbitrary_demo.py --mode interactive --plot
```

### ❌ `ParserError: Missing expression after unary operator '--'`
PowerShell 把 `--mode ...` 当成了独立命令——肯定是你换行了。**所有 flag 必须跟 `python ...` 在同一行**。

PowerShell 续行用反引号 `` ` ``，不是反斜杠 `\`：
```powershell
python python_port/run_arbitrary_demo.py --mode interactive `
    --N 8 --seed 42 --plot
```

### ❌ `=== Did not converge ===` 残差很大
ADMM 没收敛。能调的：
1. 减小 `--rho`（试 0.3 或 0.1）
2. 加 `--alpha 0.7`（under-relaxation）
3. 加 `--adaptive_rho`
4. 加 `--max_iter 500`
5. 减少 `--N`
6. 减小 `--n_int`（n_int ≤ 8 最稳定）

**调参全用上还散**：可能踩到 alpha/gamma 边界 case，不是参数问题。换 seed 试。详见 [`scaling_results.md`](scaling_results.md)。

### ❌ `Graph not connected: [X] unreachable from intersection 1`
你画的图不连通（某个路口跟其他孤立）。在 picker 里**用左键加边连起来**（让它跟其它路口正交相邻）。

### ❌ `Need at least 2 ports for any OD pair`
没放端口。最简单：picker 里**只画路口然后按 Q**，端口会自动补在所有空闲方向。

---

## 7. 文件结构（python_port/）

```
python_port/
├── README.md                  # 本文件
├── topology.py                # Topology 类 + four_int/manual/random 工厂
├── pick_topology.py           # 交互式 picker
├── generate_config.py         # 随机生成车辆 + 构造 ADMM const dict
├── build_manual_config.py     # hardcoded 4-int 配置写出 JSON
├── admm_core.py               # C-ADMM 主循环（含 over-relaxation, adaptive ρ）
├── decision_tree.py           # CR-MPC 决策树展开
├── in_admm.py                 # 路口 agent 局部 QP 子问题
├── road_agent.py              # road agent 解析更新
├── terminal_agent.py          # 终端 agent (delay cost)
├── node.py                    # 决策树节点 dataclass + factory
├── tree_utils.py              # 决策树辅助
├── constraints.py             # path-specific contention constraints
├── mpc_loop.py                # MPC 滚动优化封装
├── run_arbitrary_demo.py      # 任意拓扑 demo（CLI 主入口）
├── run_validation.py          # 4-int 验证
├── run_mpc_demo.py            # MPC demo
├── run_scaling.py             # 多种子 sweep
├── run_results.py             # 5 张图绘制（包含 plot_admm_results 公开函数）
├── run_batch_validation.py    # 批量随机配置验证
├── example_manual_topo.py     # spec 文件示例
├── scaling_results.md         # 收敛 sweep 实验记录
├── validation_config.json     # MATLAB 导出的 4-int 标准配置
└── configs/                   # 多个 random batch 场景 JSON
```

---

## 一行命令速查

```powershell
# 进目录
cd "C:\Users\robin\OneDrive\Documents\Github_file\demo_backup_0414\Distributed_schedule_for_warehouse"

# 随机模式
python python_port/run_arbitrary_demo.py --mode random --n_int 6 --N 8 --seed 42 --plot

# 交互模式
python python_port/run_arbitrary_demo.py --mode interactive --N 8 --seed 42 --plot

# 手动模式（用 example spec）
python python_port/run_arbitrary_demo.py --mode manual --spec python_port/example_manual_topo.py --N 6 --plot

# 4-int 原 paper 验证
python python_port/run_validation.py

# MPC demo
python python_port/run_mpc_demo.py

# 大网络调参组合
python python_port/run_arbitrary_demo.py --mode random --n_int 10 --N 6 --seed 42 --rho 0.3 --alpha 0.7 --adaptive_rho --max_iter 300 --plot
```
