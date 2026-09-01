# Five-intersection map

This folder contains the code specific to the second traffic map. The topology
is a 2-by-3 layout with the lower-right intersection omitted:

```text
I1 --- I2 --- I5
 |      |
I3 --- I4
```

The five intersection agents are `I1` through `I5`. Road agents connect
`I1-I2`, `I2-I5`, `I1-I3`, `I2-I4`, and `I3-I4`. The shared ADMM,
decision-tree, plotting, and solver utilities remain in the parent directory.

## Run

From the repository root, open MATLAB and run:

```matlab
run(fullfile('matlab', 'five_intersection', 'MAIN_Parallel_5int.m'))
```

`MAIN_Parallel_5int.m` calls `setup_five_intersection.m`, which adds this
module and the shared parent folder using repository-relative paths. The
default example uses the manual 10-robot case and sequential execution, so the
Parallel Computing Toolbox is not required. Set `configMode = 'random'` for
the saved random scenario settings, or `useParallel = true` when the Parallel
Computing Toolbox is available.

Requirements: MATLAB, YALMIP, and Gurobi configured as a YALMIP solver.
Generated results continue to be written under the existing parent
`BatchRuns/` directory so the preserved visualization examples remain in
their original locations.

For a disposable test run, set the `RENKE_FIVE_INT_OUTPUT_DIR` environment
variable before starting MATLAB. The module will write results there instead
of modifying the preserved `BatchRuns/` examples.
