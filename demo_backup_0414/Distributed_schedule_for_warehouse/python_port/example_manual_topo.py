"""
Example manual topology spec — pass this file via:
    python python_port/run_arbitrary_demo.py --mode manual \
        --spec python_port/example_manual_topo.py --N 6

Layout: 5-int "L" shape
        I1 - I2
        |
        I3 - I4 - I5
"""

name = 'example_L5'

coords = {
    1: (0, 1),
    2: (1, 1),
    3: (0, 0),
    4: (1, 0),
    5: (2, 0),
}

ports = [
    (1, 'N'), (1, 'W'),
    (2, 'N'), (2, 'E'),
    (3, 'W'), (3, 'S'),
    (4, 'S'),
    (5, 'N'), (5, 'E'), (5, 'S'),
]
