"""
    Author: Zhanluo Zhang
    Author E-mail: zhangzhanluo@outlook.com
    Version: v1.0
    Created Date: 20210531
    Description: the mixed integer programming model of the HFSP with total flow time minimization objective
"""

import gurobipy as grb
# from part_00_generate_processing_time import generate_processing_time

I = [2, 2]
p = [[4, 3, 6, 4, 4, 2],
     [5, 5, 3, 3, 3, 3]]
n_stages = len(p)
n_jobs = len(p[0])
assert n_stages == len(I)

goal = 'c_j'
assert goal in ['c_max', 'c_j']

try:

    # Create a new model
    m = grb.Model("mip_hfsp")

    # Create variables
    s = m.addVars(n_stages, n_jobs, lb=0, vtype=grb.GRB.CONTINUOUS, name='s')
    x_indices = []
    for j in range(n_jobs):
        for k in range(n_stages):
            for i in range(I[k]):
                x_indices.append((j, i, k))
    x = m.addVars(x_indices, vtype=grb.GRB.BINARY, name='x')
    y = m.addVars(n_stages, n_jobs, n_jobs, vtype=grb.GRB.BINARY, name='y')

    # Add constraints
    if goal == 'c_j':
        c = m.addVars(n_jobs, vtype=grb.GRB.CONTINUOUS, name='c', obj=1)
        m.addConstrs([s[n_stages - 1, j] + p[n_stages - 1][j] == c[j] for j in range(n_jobs)],
                     name='(2)')
    else:
        c_max = m.addVar(vtype=grb.GRB.CONTINUOUS, name='c_max', obj=1)
        m.addConstrs([s[n_stages - 1, j] + p[n_stages - 1][j] <= c_max for j in range(n_jobs)],
                     name='(9)')
    m.addConstrs([x.sum(j, '*', k) == 1 for j in range(n_jobs) for k in range(n_stages)],
                 name='(3)')
    m.addConstrs([s[k, j] + p[k][j] <= s[k + 1, j] for j in range(n_jobs) for k in range(n_stages - 1)],
                 name='(4)')
    m.addConstrs(
        [s[k, j] - (s[k, r] + p[k][r]) +
         grb.GRB.INFINITY * (2 + y[k, j, r] - x[j, i, k] - x[r, i, k]) >= 0
         for j in range(n_jobs)
         for r in range(n_jobs)
         for k in range(n_stages - 1)
         for i in range(I[k])
         if j < r],
        name='(5)')
    m.addConstrs(
        [s[k, r] - (s[k, j] + p[k][j]) +
         grb.GRB.INFINITY * (3 - y[k, j, r] - x[j, i, k] - x[r, i, k]) >= 0
         for j in range(n_jobs)
         for r in range(n_jobs)
         for k in range(n_stages - 1)
         for i in range(I[k])
         if j < r],
        name='(6)')

    # Set objective
    m.modelSense = grb.GRB.MINIMIZE

    m.optimize()

    for v in m.getVars():
        print(v.varName, v.x)

    print('Obj:', m.objVal)

except grb.GurobiError:
    print('Error reported')
