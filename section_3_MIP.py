"""
    Author: Zhanluo Zhang
    Author E-mail: zhangzhanluo@outlook.com
    Version: v1.0
    Created Date: 20210531
    Description: the mixed integer programming model of the HFSP with total flow time minimization objective
"""
import time
import gurobipy as grb
from section_0_preparing import HFSPInstance, draw_gant_chart


def mip(instance, goal, time_limit=600, gant_path=None):
    """
    solve the instance using mixed integer programming model
    :param instance: case, (processing time, number of machines for each stage). processing_time: stage, job
    :param goal: c_j or c_max
    :param time_limit: time limit for solving the mip model
    :param gant_path: path to save the gant chart of the result
    :return: objective value, model building time, model solving time
    """
    assert goal in ['c_j', 'c_max']
    n_stages = instance.n_stages
    n_jobs = instance.n_jobs
    p = instance.processing_time
    I = instance.stages_machine
    Q = 100000.0  # a very large number
    try:
        start_time = time.time()
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

        # Add variables and constraints according to the goal
        if goal == 'c_j':
            c = m.addVars(n_jobs, vtype=grb.GRB.CONTINUOUS, name='c', obj=1)
            m.addConstrs((s[n_stages - 1, j] + p[n_stages - 1][j] == c[j] for j in range(n_jobs)),
                         name='(2)')
        else:
            c_max = m.addVar(vtype=grb.GRB.CONTINUOUS, name='c_max', obj=1)
            m.addConstrs((s[n_stages - 1, j] + p[n_stages - 1][j] <= c_max for j in range(n_jobs)),
                         name='(9)')
        m.addConstrs((x.sum(j, '*', k) == 1 for j in range(n_jobs) for k in range(n_stages)),
                     name='(3)')
        m.addConstrs((s[k, j] + p[k][j] <= s[k + 1, j] for j in range(n_jobs) for k in range(n_stages - 1)),
                     name='(4)')

        # Here, both solution 1 and solution 2 are right
        # %% solution 1 (the one proposed by the article) start
        m.addConstrs(
            (s[k, j] - (s[k, r] + p[k][r]) +
             Q * (2 + y[k, j, r] - x[j, i, k] - x[r, i, k]) >= 0
             for j in range(n_jobs)
             for r in range(n_jobs)
             for k in range(n_stages)
             for i in range(I[k])
             if j < r),
            name='(5)')
        m.addConstrs(
            (s[k, r] - (s[k, j] + p[k][j]) +
             Q * (3 - y[k, j, r] - x[j, i, k] - x[r, i, k]) >= 0
             for j in range(n_jobs)
             for r in range(n_jobs)
             for k in range(n_stages)
             for i in range(I[k])
             if j < r),
            name='(6)')
        # %% solution 1 end
        # %% solution 2 start
        # m.addConstrs(
        #     (s[k, j] - (s[k, r] + p[k][r]) +
        #      Q * (2 + y[k, j, r] - x[j, i, k] - x[r, i, k]) >= 0
        #      for j in range(n_jobs)
        #      for r in range(n_jobs)
        #      for k in range(n_stages)
        #      for i in range(I[k])),
        #     name='(5)')
        # m.addConstrs((y[k, j, r] + y[k, r, j] == 1
        #               for j in range(n_jobs)
        #               for r in range(n_jobs)
        #               for k in range(n_stages)
        #               if j != r))
        # %% solution 2 end

        # Set objective
        m.modelSense = grb.GRB.MINIMIZE
        model_building_time = time.time() - start_time

        # solve the model
        m.setParam('TimeLimit', time_limit)
        m.optimize()
        model_solving_time = time.time() - model_building_time

        # print results
        for v in m.getVars():
            print(v.varName, v.x)
        print('Obj:', m.objVal)

        # draw gant chart
        if gant_path is not None:
            job_machine_info = []
            for j in range(n_jobs):
                for k in range(n_stages):
                    machine_id = [m.x for m in x.select(j, '*', k)].index(1.0)
                    job_machine_info.append((j + 1, '{}-{}'.format(k+1, machine_id+1), s[k, j].x, p[k][j]))
            draw_gant_chart(job_machine_info, title='Gant Chart for {} Instance (MIP)'.format(instance.name),
                            save_path=gant_path)

        return m.objVal, model_building_time, model_solving_time

    except grb.GurobiError:
        print('Error reported')
        return 0, 0, 0


if __name__ == '__main__':
    case = HFSPInstance(default=True)
    optimal_solution = mip(case, goal='c_j', gant_path='02_Results/30_MIP/Pics/')
    case_1 = HFSPInstance(n_jobs=6, n_stages=10, machine_layout='e')
    optimal_solution_1 = mip(case_1, goal='c_j', gant_path='02_Results/30_MIP/Pics/')
