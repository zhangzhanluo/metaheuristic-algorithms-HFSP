"""
    Author: Zhanluo Zhang
    Author E-mail: zhangzhanluo@outlook.com
    Version: v1.0
    Created Date: 20210706
    Description: 使用instance j40c5e4对所写代码进行验证，结果发现错误！错误详在section_4_metaheuristic_algorithm文件中的
                 forward_scheduling_approach函数中的注释。
"""
from matplotlib import pyplot as plt
from section_4_metaheuristic_algorithm import HFSPInstance, MetaheuristicAlgorithmSolver

# instance j40c5e4 and its best solution provided by the article
original_processing_time = [[59, 26, 95, 27, 72],
                            [18, 69, 8, 24, 10],
                            [58, 47, 33, 4, 3],
                            [62, 62, 100, 88, 4],
                            [92, 85, 43, 67, 54],
                            [2, 64, 18, 98, 82],
                            [73, 76, 83, 82, 90],
                            [68, 67, 59, 78, 82],
                            [39, 87, 12, 97, 36],
                            [97, 88, 23, 18, 42],
                            [75, 22, 87, 49, 21],
                            [19, 77, 54, 69, 21],
                            [50, 76, 93, 65, 39],
                            [63, 10, 44, 71, 88],
                            [78, 46, 76, 80, 28],
                            [10, 19, 82, 20, 8],
                            [54, 97, 78, 7, 71],
                            [86, 87, 95, 66, 93],
                            [61, 81, 1, 2, 16],
                            [96, 72, 51, 27, 11],
                            [94, 76, 99, 61, 44],
                            [42, 98, 98, 49, 72],
                            [78, 47, 94, 13, 41],
                            [74, 9, 89, 1, 64],
                            [32, 55, 84, 19, 39],
                            [41, 94, 21, 82, 1],
                            [69, 20, 86, 26, 1],
                            [81, 72, 99, 68, 74],
                            [11, 67, 96, 9, 93],
                            [15, 19, 35, 27, 27],
                            [10, 13, 6, 43, 66],
                            [93, 38, 3, 90, 33],
                            [54, 42, 58, 33, 51],
                            [34, 33, 21, 24, 61],
                            [26, 77, 81, 94, 46],
                            [45, 21, 14, 5, 76],
                            [24, 8, 9, 69, 86],
                            [93, 73, 59, 39, 96],
                            [16, 24, 72, 80, 23],
                            [20, 46, 73, 42, 83]]
processing_time = [[row[i] for row in original_processing_time] for i in range(len(original_processing_time[0]))]
hfsp = HFSPInstance(n_jobs=40, n_stages=5)
hfsp.stages_machine = [5, 5, 4, 3, 3]
hfsp.processing_time = processing_time
hfsp_solver = MetaheuristicAlgorithmSolver(hfsp, goal='C_max')
solution = '30	5	29	23	35	15	33	36	1	39	0	13	7	8	34	37	' \
           '6	24	28	38	17	4	20	16	12	27	31	11	21	9	22	32	3	14	10	18	19	2	26	25'
solution = solution.split('\t')
solution = [int(x) + 1 for x in solution]
result = hfsp_solver.forward_scheduling_approach(solution)

all_machines = []
for i in range(5):
    for j in range(hfsp.stages_machine[i]):
        all_machines.append('{}-{}'.format(i, j))

plt.figure(figsize=(16, 9), dpi=300)
for i, job in enumerate(solution):
    for j in range(5):
        y = all_machines.index('{}-{}'.format(j, result['x_jki'][i][j].index(1)))
        width = processing_time[j][job - 1]
        left = result['s_kj'][j][i]
        job_id = job - 1
        plt.barh(y,
                 width=width,
                 height=0.8,
                 left=left,
                 facecolor='white',
                 edgecolor='black')
        plt.text(left + width / 2, y, job_id, ha='center', va='center', fontsize=7)
plt.xlim([0, 800])
plt.xticks(range(0, 801, 20))
plt.grid(axis='x', which='both')
plt.xlabel('Time (s)')
plt.ylabel('Stage-Machine')
plt.yticks(range(len(all_machines)), all_machines)
plt.tight_layout()
plt.savefig('02_Results/Pics/IGT_CMax_j40c5e4.png')
plt.show()
