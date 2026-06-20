from python_port.topology import manual_topology

coords = {1: (0, 1), 2: (1, 1), 3: (1, 0), 4: (0, 0)}
ports = [(1, 'N'), (1, 'W'), (2, 'N'), (2, 'E'),
         (3, 'E'), (3, 'S'), (4, 'S'), (4, 'W')]

t1 = manual_topology(coords=coords, ports=ports, name='full')
print('Full 2x2:')
print('  n_road =', t1.n_road, ' OD =', len(t1.route_dict))
print('  route port1 -> port4: ints =', t1.route_dict[(1, 4)]['ints'])

t2 = manual_topology(coords=coords, ports=ports,
                     excluded_edges=[{1, 2}], name='no_I1_I2')
print()
print('With excluded {1,2}:')
print('  n_road =', t2.n_road, ' OD =', len(t2.route_dict))
print('  route port1 -> port4: ints =', t2.route_dict[(1, 4)]['ints'])
print('  excluded_edges stored:', t2.excluded_edges)

# Fully ADMM-able?
from python_port.generate_config import generate_random_config
from python_port.admm_core import run_admm_core
const, ap = generate_random_config(N=6, seed=11, max_per_int=6, Dt=2.0,
                                    max_iter=200, topology=t2)
const['useParallel'] = False
const['verbose'] = False
res = run_admm_core(const, ap)
*_, k, _, _, _ = res
print()
print('ADMM on the topology with excluded edge: k_conv =', k,
      ' final cost =', round(float(res[5][k - 1]), 4))
