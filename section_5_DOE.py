"""
    Author: Zhanluo Zhang
    Author E-mail: zhangzhanluo@outlook.com
    Version: v1.0
    Created Date: 
    Description: 
"""
import os
import random
from itertools import product
from section_4_metaheuristic_algorithm import HFSPInstance, MetaheuristicAlgorithmSolver

goal = 'TFT'
jobs = [30, 40, 50]
n_stages = 5
instance_size = 3

# %% DOE for IGT_ALL parameters
dSs = [2, 5, 8]
tPs = [0.1, 0.3, 0.5]
jPs = [0, 0.2, 0.4, 0.6, 0.8, 1.0]

result_dir = '02_Results/50_DOE/'
if not os.path.exists(result_dir):
    os.makedirs(result_dir)

file_name = result_dir+'IG_ALL_DOE_result.csv'
if not os.path.exists(file_name):
    with open(file_name, 'w') as f:
        results = ['CaseName'] + ['NJob', 'dS', 'tP', 'jP'] + ['Case{}'.format(x+1) for x in range(instance_size)]
        f.write(','.join(results)+'\n')

parameter_settings = list(product(jobs, dSs, tPs, jPs))
random.seed(1)
random.shuffle(parameter_settings)
random.seed(None)
for parameter_setting in parameter_settings:
    n_jobs, dS, tP, jP = parameter_setting
    case_name = '#'.join([str(x) for x in parameter_setting])
    print(case_name)
    case_goals = []
    for j in range(instance_size):
        print('Case {}'.format(j))
        hfsp_instance = HFSPInstance(n_jobs=n_jobs, n_stages=n_stages, machine_layout='e', random_seed=j)
        ma_solver = MetaheuristicAlgorithmSolver(instance=hfsp_instance, goal=goal)
        _, ig_all_result = ma_solver.IG_algorithm(variant='IG_ALL', d_S=dS, tP=tP, jP=jP, random_seed=j,
                                                  show_result=False, save_result=False)
        case_goals.append(ig_all_result[goal])
    with open(file_name, 'a') as f:
        results = [case_name] + [str(x) for x in parameter_setting] + [str(case_goal) for case_goal in case_goals]
        f.write(','.join(results)+'\n')
    break
