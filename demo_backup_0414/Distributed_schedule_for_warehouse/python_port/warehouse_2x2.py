"""
Warehouse 2x2 4-intersection topology (matches Traffic_Demo HTML layout).

Layout (col, row):
    P4(0,1) ── P3(1,1)
       │          │
    P1(0,0) ── P2(1,0)

External ports per intersection (8 total):
    P1: S, W   P2: S, E   P3: N, E   P4: N, W

Run:
    python python_port/run_arbitrary_demo.py --mode manual \
        --spec python_port/warehouse_2x2.py --N 20 --seed 4101 \
        --Dt 3.0 --T_val 2.0 --max_iter 300
"""

name = 'warehouse_2x2'

coords = {
    1: (0, 0),
    2: (1, 0),
    3: (1, 1),
    4: (0, 1),
}

# Port order MUST match the demo HTML's hard-coded layout
# (warehouse_amr_demo_test.html, ASCII diagram around line 2433):
#       ⑥↓  ⑤↓
#   ⑦→ ┌──────────┐ ←④
#       │ P4    P3 │
#   ⑧→ │ P1    P2 │ ←③
#       └──────────┘
#       ↑①   ↑②
ports = [
    (1, 'S'),    # 1: P1 south (entrance ①)
    (2, 'S'),    # 2: P2 south (entrance ②)
    (2, 'E'),    # 3: P2 east  (entrance ③)
    (3, 'E'),    # 4: P3 east  (entrance ④)
    (3, 'N'),    # 5: P3 north (entrance ⑤)
    (4, 'N'),    # 6: P4 north (entrance ⑥)
    (4, 'W'),    # 7: P4 west  (entrance ⑦)
    (1, 'W'),    # 8: P1 west  (entrance ⑧)
]
