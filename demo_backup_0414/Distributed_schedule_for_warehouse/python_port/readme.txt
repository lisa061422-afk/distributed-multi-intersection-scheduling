========================================
仓库调度 Python 端 — 运行说明
========================================

【安装依赖】（只需一次）
    pip install numpy scipy

【运行方式】
必须在仓库根目录（Distributed_schedule_for_warehouse/）下运行，
不能在 python_port/ 内部运行。

打开 VS Code 终端（Ctrl+`），然后：

    cd "C:\Users\robin\OneDrive\Documents\Github_file\demo_backup_0414\Distributed_schedule_for_warehouse"

----------------------------------------
1. 数值验证（Python vs MATLAB 结果对比）
----------------------------------------
    python python_port/run_validation.py

输出：顺序/并行运行的迭代数、耗时、收敛代价。

----------------------------------------
2. MPC 在线演示（三个场景）
----------------------------------------
    python python_port/run_mpc_demo.py

场景 A：静态 ADMM（t=0 单次求解）
场景 B：MPC 滚动优化，无扰动
场景 C：MPC 滚动优化，速度扰动 std=0.1 m/s

输出：每步迭代数、求解时间、延迟代价，以及
      "dt ≥ Xs 可行" 的 MPC 时间步建议。
