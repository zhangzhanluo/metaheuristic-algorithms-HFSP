"""
    Author: Zhanluo Zhang
    Author E-mail: zhangzhanluo@outlook.com
    Version: v1.0
    Created Date: 20210602
    Description: Forward scheduling approach and calculate C_max and C_j
"""
import random
from section_0_preparing import HFSPInstance, draw_gant_chart


class MetaheuristicAlgorithmSolver:
    def __init__(self, instance, goal='TFT'):
        """
        example：
            ma_solver = MetaheuristicAlgorithmSolver(hfsp_instance)
            neh_solution_result = ma_solver.NEH_heuristic(goal='TFT', show_result=True, save_result=True)

        :param instance: HFSP instance
        :param goal: TFT or C_max
        """
        self.instance = instance
        self.n_jobs = instance.n_jobs
        self.n_stages = instance.n_stages
        self.p_kj = instance.processing_time
        self.I_k = instance.stages_machine
        self.TFT = None
        self.c_max = None
        self.goal = goal
        # The threshold value α ∈ [0, 1] limits the size of RCL and regulates balance between
        # greedy and randomized procedures.
        self.grasp_alpha = 0.1

    def forward_scheduling_approach(self, pi):
        """
        In forward scheduling, the jobs are assigned to the most available machine in the first stage with the order of
        initial solution π. Afterwards, in each following stage, the jobs are ordered with regard to their release times
        from the previous stage and assigned to the most available machine in that order. Note that the order of jobs
        can be different in each stage. In this manner, a complete schedule can be found by using only an initial
        solution π.

        :param pi: solution, can be part of the jobs
        :return: x_jki, s_kj, c_max, c_j. x_jki -> 1 if job pi[j] is processed at machine i at stage k, 0 otherwise;
        s_kj -> Starting time of job pi[j] at stage k. Note that in the article it is x_jik instead of x_jki.
        x_jki is better when coding it.
        """
        system_clock = 0
        finish_flag = 1e6
        jobs_finishing_time = [0.0 for _ in pi]  # 记录任务完成时间
        jobs_stage = [-1 for _ in pi]  # 记录任务阶段
        machines_status = [[0.0 for _ in range(self.I_k[i])] for i in range(self.n_stages)]  # 记录机器正在处理的任务完成时间或者空闲
        x_jki = [[[0 for _ in range(self.I_k[k])] for k in range(self.n_stages)] for _ in range(len(pi))]
        s_kj = [[-1 for _ in range(len(pi))] for _ in range(self.n_stages)]
        while True:
            for j in range(len(pi)):
                # 对所有任务进行遍历，判断是否可以结束以及是否可以开始
                if jobs_finishing_time[j] <= system_clock:  # 任务需要转移到下一阶段
                    k = jobs_stage[j]
                    if k >= 0:  # 初始状态下没有机器在工作
                        current_machine = x_jki[j][k].index(1)
                        # 有个问题，当你上一次结束之后，下阶段没有可用机器，就进入buffer，当前的机器空闲，后续的机器工作；但如果此时下方机器都被占用了，就会等待；
                        # 稍后，系统时钟拨到了下一个时刻，有个机器完成任务了，前面的机器抢占机器，但是此时，他会再把当前的机器置零，
                        # 相当于把别人的工作结束了，可是下一个人的工作还停留在第一阶段的记忆。如果这个值很大，就会发生，当其余任务都完成了，没有机器再叫这个被遗忘的任务了。
                        # 总结就是当某个任务结束时间非常长，且碰巧被前面的任务把自己的机器状态抹去了，就会发生问题。因此有了一下的判断
                        if machines_status[k][current_machine] == system_clock:  # 任务只有在结束的时刻才对自己的机器状态有话语权！
                            machines_status[k][current_machine] = 0.0  # 任务时间到了，就可以结束了，当前机器占用状态重置，这对应于无限buffer的情况
                    if k == self.n_stages - 1:  # 是倒数第一阶段，则直接标记任务离开系统
                        jobs_finishing_time[j] = finish_flag  # 用一个无穷大的数字表示任务已经永远结束
                    else:  # 不是最后一个阶段，需要进行下一步开始判断
                        if min(machines_status[k + 1]) == 0.0:  # 下一个阶段有可用机器
                            # 任务转移至下一个阶段，任务的结束时间进行更新
                            jobs_stage[j] = k + 1
                            # 注意！操作时间的矩阵，j的含义与此处j的含义需要用solution解读一下（pi[j]-1）
                            jobs_finishing_time[j] = system_clock + self.p_kj[k + 1][pi[j] - 1]
                            s_kj[k + 1][j] = system_clock
                            # 对机器状态进行维护，下一阶段机器被占用
                            available_machine = machines_status[k + 1].index(0.0)
                            machines_status[k + 1][available_machine] = jobs_finishing_time[j]
                            x_jki[j][k + 1][available_machine] = 1
            # 调整系统时钟，调整为下一个机器完成一个任务的时刻
            all_working_machine_status = []
            for stage_machines_status in machines_status:
                stage_working_machines_status = [x for x in stage_machines_status if x > 0]
                all_working_machine_status.extend(stage_working_machines_status)
            if len(all_working_machine_status) == 0:  # 当所有的机器都空闲后，系统所有的任务都完成，结束
                break
            system_clock = min(all_working_machine_status)  # 将系统时钟调整到下一个任务结束的时刻
        all_c_j = [s_kj[-1][j] + self.p_kj[-1][j] for j in range(len(s_kj[0]))]
        return {'x_jki': x_jki, 's_kj': s_kj, 'TFT': round(sum(all_c_j), 1), 'C_max': round(max(all_c_j), 1)}

    def draw_fs_result_gant_chart(self, pi, method=None, show_chart=True, save_result=None):
        """
        Draw Gant chart for forward scheduling result.

        :param pi: solution
        :param method: how you get pi
        :param show_chart: show it
        :param save_result: save or not
        :return: nothing
        """
        forward_approach_result = self.forward_scheduling_approach(pi)
        x_jki, s_kj = forward_approach_result['x_jki'], forward_approach_result['s_kj']
        job_machine_info = []
        for j in range(len(pi)):
            for k in range(self.n_stages):
                machine_id = x_jki[j][k].index(1)
                job_machine_info.append(
                    (pi[j], '{}-{}'.format(k + 1, machine_id + 1), s_kj[k][j],
                     self.p_kj[k][pi[j] - 1]))
        if save_result:
            save_path = '02_Results/40_metaheuristic_algorithm/Pics/'
        else:
            save_path = None
        draw_gant_chart(job_machine_info, instance_name=self.instance.name, method=method, goal=self.goal,
                        TFT=forward_approach_result['TFT'], C_max=forward_approach_result['C_max'],
                        show_chart=show_chart, save_chart=save_path)

    def sort_the_jobs(self):
        """
        stage 1 of NEH heuristic.
        the sum of the processing times on all stages (TPj) is calculated for each job j ∈ J.
        Then, jobs are sorted in decreasing order of TPj.

        :return: sorted jobs
        """
        TP = [sum([self.p_kj[k][j] for k in range(self.instance.n_stages)]) for j in range(self.n_jobs)]
        tp_jobs = [(TP[i], i + 1) for i in range(self.n_jobs)]  # 这里给出了每个任务的名字，就是他们对应的顺序
        tp_jobs.sort(key=lambda x: x[0], reverse=True)
        sorted_jobs = [tp_job[1] for tp_job in tp_jobs]
        return sorted_jobs

    def NEH_heuristic(self, guiding_solution=None, show_result=True, save_result=False):
        """
        give a solution, and show it if ordered.


        :param show_result: show result
        :param guiding_solution: a guiding solution
        :param save_result: save result to 02_Results/Pics/
        :return: a neh solution and its forward scheduling result
        """
        if guiding_solution is None:
            guiding_solution = self.sort_the_jobs()
        pi = [guiding_solution[0]]
        for i in range(1, self.n_jobs):
            fitness_ls = []
            for j in range(len(pi) + 1):
                current_pi = pi.copy()
                current_pi.insert(j, guiding_solution[i])
                forward_scheduling_result = self.forward_scheduling_approach(current_pi)
                fitness_ls.append(forward_scheduling_result[self.goal])
            pi.insert(fitness_ls.index(min(fitness_ls)), guiding_solution[i])
        solution_result = self.forward_scheduling_approach(pi)
        self.draw_fs_result_gant_chart(pi, method='NEH', show_chart=show_result, save_result=save_result)
        return pi, solution_result

    def GRASP(self, pi_star, h, random_seed=None):
        """
        Greedy Randomized Adaptive Search Procedure。

        :param pi_star: initial order of jobs, in this study it is always the result of self.sort_the_jobs()
        :param h: decide the leading job for GRASP
        :param random_seed: control random seed for random select
        :return:
        """
        random.seed(random_seed)
        U = [i + 1 for i in range(self.n_jobs)]
        pi_1 = pi_star[h]
        U.remove(pi_1)
        pi = [pi_1]
        for _ in range(1, self.n_jobs):
            CF_list = []
            for j in U:
                current_pi = pi.copy()
                current_pi.append(j)
                current_pi_result = self.forward_scheduling_approach(current_pi)
                CF_list.append(current_pi_result[self.goal])
            CF_min, CF_max = min(CF_list), max(CF_list)
            RCL = []
            for i, j in enumerate(U):
                if CF_list[i] - CF_min <= self.grasp_alpha * (CF_max - CF_min):
                    RCL.append(j)
            pi.append(random.choice(RCL))
            U.remove(pi[-1])
        random.seed(None)
        return pi

    def GRASP_NEH_heuristic(self, x=None, show_result=True, save_result=False, random_seed=None):
        """
        GRASP_NEH(x) heuristic.

        :param x: We fix the number of solutions x as n in this study.
        :param show_result: show result
        :param save_result: save result
        :param random_seed: random state control for GRASP.
        :return: best solution and its forward scheduling result
        """
        if x is None:
            x = self.n_jobs
        pi_star = self.sort_the_jobs()
        pi_list = []
        pi_goal_values = []
        for h in range(0, x):
            pi_1 = self.GRASP(pi_star, h, random_seed=random_seed)
            pi_1_result = self.forward_scheduling_approach(pi_1)
            pi_2, _ = self.NEH_heuristic(guiding_solution=pi_1, show_result=False, save_result=False)
            pi_2_result = self.forward_scheduling_approach(pi_2)
            if pi_2_result[self.goal] < pi_1_result[self.goal]:
                pi_list.append(pi_2)
                pi_goal_values.append(pi_2_result[self.goal])
            else:
                pi_list.append(pi_1)
                pi_goal_values.append(pi_2_result[self.goal])
        best_pi = pi_list[pi_goal_values.index(min(pi_goal_values))]
        best_pi_result = self.forward_scheduling_approach(best_pi)
        self.draw_fs_result_gant_chart(best_pi, method='GRASP_NEH', show_chart=show_result, save_result=save_result)
        return best_pi, best_pi_result


if __name__ == '__main__':
    hfsp_instances = [HFSPInstance(default=True), HFSPInstance(n_jobs=6, n_stages=3, random_seed=1)]
    for g in ['TFT', 'C_max']:
        for hfsp_instance in hfsp_instances:
            ma_solver = MetaheuristicAlgorithmSolver(hfsp_instance, goal=g)
            neh_solution_result = ma_solver.NEH_heuristic(show_result=True, save_result=True)
            grasp_neh_solution_result = ma_solver.GRASP_NEH_heuristic(show_result=True, save_result=True)
