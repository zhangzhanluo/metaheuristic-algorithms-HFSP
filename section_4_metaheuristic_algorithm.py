"""
    Author: Zhanluo Zhang
    Author E-mail: zhangzhanluo@outlook.com
    Version: v1.0
    Created Date: 20210602
    Description: Forward scheduling approach and calculate C_max and C_j
"""
import time
import math
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
        self.time_limit = 30 * self.n_jobs * self.n_stages / 1000

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
        all_c_j = [s_kj[-1][j] + self.p_kj[-1][pi[j] - 1] for j in range(len(s_kj[0]))]
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

    def NEH_insertion(self, pi, job):
        """
        Try all possible position and insert the job into the best position.

        :param pi: a not completed solution
        :param job: the inserted job
        :return: pi+job for the best position
        """
        assert job not in pi
        fitness_ls = []
        for j in range(len(pi) + 1):
            current_pi = pi.copy()
            current_pi.insert(j, job)
            forward_scheduling_result = self.forward_scheduling_approach(current_pi)
            fitness_ls.append(forward_scheduling_result[self.goal])
        pi.insert(fitness_ls.index(min(fitness_ls)), job)
        return pi

    def NEH_heuristic(self, guiding_solution=None, show_result=True, save_result=False):
        """
        Nawaz, Enscore, Ham (NEH) Algorithm, 1983. Sort first and insert the job one by one.

        :param guiding_solution: a guiding solution
        :param show_result: show result
        :param save_result: save result to 02_Results/Pics/
        :return: a NEH solution and its forward scheduling result
        """
        if guiding_solution is None:
            guiding_solution = self.sort_the_jobs()
        pi = [guiding_solution[0]]
        for i in range(1, self.n_jobs):
            pi = self.NEH_insertion(pi, guiding_solution[i])
        solution_result = self.forward_scheduling_approach(pi)
        self.draw_fs_result_gant_chart(pi, method='NEH', show_chart=show_result, save_result=save_result)
        return pi, solution_result

    def GRASP(self, pi_star, h, random_seed=None):
        """
        Greedy Randomized Adaptive Search Procedure, Feo and Resende (1995).
        From a set of candidate jobs to be sequenced, the next job to be appended is selected using a cost function.
        The cost of each candidate is computed and a restricted candidate list (RCL) is constructed, which holds
        jobs with a cost no larger than cfmin + α (cfmax−cfmin), where cfmin and cfmax are the minimum and maximum
        costs. The threshold value α ∈ [0, 1] limits the size of RCL and regulates balance between greedy and
        randomized procedures.

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
        GRASP_NEH(x) heuristic. Use all the job as leading job, and then get a GRASP solution, and then improve it by
        NEH algorithm, and finally choose the best solution among all the leading job.

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

    @staticmethod
    def destruction(pi, dS, random_seed=None):
        """
        Chooses randomly, without repetition d jobs. These d jobs are then removed from pi in the order in which they
        were chosen.The result of this procedure are two subsequences,the first being the partial sequence pi_D with
        n-d jobs, that is the sequence after the removal of d jobs, and the second being a sequence of d jobs, which
        we denote as pi_R.
        :param pi: solution
        :param dS: number of jobs to be removed from pi
        :param random_seed:
        :return: pi_D -> destruction, pi_R -> repair
        """
        random.seed(random_seed)
        pick_flags = [1 for _ in range(dS)] + [0 for _ in range(len(pi) - dS)]
        random.shuffle(pick_flags)
        random.seed(None)
        pi_R, pi_D = [], []
        for i, pick_flag in enumerate(pick_flags):
            if pick_flag:
                pi_R.append(pi[i])
            else:
                pi_D.append(pi[i])
        return pi_D, pi_R

    def construction(self, pi_D, pi_R):
        """
        The construction phase consists in the application of step 3 of the NEH heuristic until a complete sequence of
        all n jobs is obtained. The construction phase starts with subsequence pi_D and performs d steps in which the
        jobs in pi_R are reinserted into pi_D. Hence, we start with pi_D and insert the first job of pi_R, pi_R(1),
        into all possible n - d + 1 positions of pi_D. The best position for pi_R(1) in the augmented pi_D sequence is
        the one that yields the smallest Cmax. This process is iterated until pi_R is empty.

        :param pi_D: destruct set
        :param pi_R: repair set
        :return: a complete solution
        """
        for j in pi_R:
            pi_D = self.NEH_insertion(pi_D, j)
        return pi_D

    def first_improvement_insertion_local_search(self, pi):
        """
        Local search aims to further enhance solution quality. For each job πi, the procedure inserts job πi into
        all possible positions of solution π. When the most improving insertion position is found, job πi is
        inserted into that position. These steps are repeated for all jobs. If an improvement is found, the local
        search is re-run until no better solution is obtained.

        :param pi: original solution
        :return: solution improved by local search
        """
        improvement_flag = True
        pi_goal = self.forward_scheduling_approach(pi)[self.goal]
        while improvement_flag:
            improvement_flag = False
            for i in range(len(pi)):
                current_pi = pi.copy()
                current_pi.remove(pi[i])
                all_new_pi = []
                all_fitness = []
                for j in range(len(current_pi) + 1):
                    new_pi = current_pi.copy()
                    new_pi.insert(j, pi[i])
                    all_new_pi.append(new_pi)
                    all_fitness.append(self.forward_scheduling_approach(new_pi)[self.goal])
                best_new_pi = all_new_pi[all_fitness.index(min(all_fitness))]
                if best_new_pi != pi and min(all_fitness) < pi_goal:
                    pi = best_new_pi
                    pi_goal = self.forward_scheduling_approach(pi)[self.goal]
                    improvement_flag = True
        return pi

    def r_local_search(self, algorithm, pi, pi_best):
        """
        Referenced Insertion Scheme (RIS) and Referenced Swap Scheme (RSS) algorithms. Give a better pi by local search.

        :param algorithm: RIS or RSS
        :param pi: current solution
        :param pi_best: best known solution
        :return: maybe a better solution
        """
        assert algorithm in ['RIS', 'RSS']
        pi_R = pi_best.copy()
        i = 0
        pos = 0
        n = len(pi)
        pi_goal = self.forward_scheduling_approach(pi)[self.goal]
        while i < n:
            k = 0
            while pi[k] != pi_R[pos]:
                k = k + 1
            pos += 1
            if pos == n:
                pos = 0
            all_fitness = []
            all_new_pi = []
            pi_k_index = pi.index(pi[k])
            if algorithm == 'RIS':
                pi_remove_k = pi.copy()
                pi_remove_k.remove(pi[k])
                for j in range(n):
                    if j != pi_k_index:
                        new_pi = pi_remove_k.copy()
                        new_pi.insert(j, pi[k])
                        all_new_pi.append(new_pi)
                        all_fitness.append(self.forward_scheduling_approach(new_pi)[self.goal])
            else:
                for j in range(n):
                    if j != pi_k_index:
                        new_pi = pi.copy()
                        temp = new_pi[pi_k_index]
                        new_pi[pi_k_index] = new_pi[j]
                        new_pi[j] = temp
                        all_new_pi.append(new_pi)
                        all_fitness.append(self.forward_scheduling_approach(new_pi)[self.goal])
            best_new_pi = all_new_pi[all_fitness.index(min(all_fitness))]
            if min(all_fitness) < pi_goal:
                pi = best_new_pi.copy()
                pi_goal = self.forward_scheduling_approach(pi)[self.goal]
                i = 1
            else:
                i += 1
        return pi

    @staticmethod
    def remove_block(pi, b, random_seed=None):
        random.seed(random_seed)
        start_point = random.randint(1, len(pi) - 2)  # make sure that the block will not be the whole solution
        pi_b = pi[start_point: start_point + b]
        pi_p = pi[:start_point] + pi[start_point + b:]
        random.seed(None)
        return pi_p, pi_b

    def IG_algorithm(self, variant, d_S=2, tP=0.5, jP=0.4, random_seed=None, show_result=True,
                     save_result=False):
        """
        Iterated greedy algorithm proposed by Ruiz and Stützle (2007)

        :param variant: variant of IG algorithms: IG_RS, IG_GR, IGT, IGT_ALL
        :param d_S: dS jobs are randomly removed from current solution π and are re-inserted into the partial solution
                in a sequential manner. According DOE dS is set to 2 for IG_RS, IG_GR.
        :param tP: τP is a parameter to be adjusted and is used to calculated the constant temperature T. According to
                DOE tP is set to 0.5 for IG_RS, IG_GR, IGT, IGT_ALL.
        :param jP: jump probability. According to DOE tP is set to 0.4 for IG_RS, IG_GR, IGT, IGT_ALL.
        :param random_seed: random seed for deconstruction and construction
        :param show_result: show result
        :param save_result: save result
        :return: best solution and its forward scheduling result
        """
        random.seed(random_seed)
        r = random.uniform(0, 1)
        random.seed(None)
        T = sum([sum(self.p_kj[k]) for k in range(self.n_stages)]) / (10 * self.n_jobs * self.n_stages) * tP
        if variant in ['IG_RS']:
            pi_0, _ = self.NEH_heuristic(show_result=False, save_result=False)
        elif variant in ['IG_GR', 'IGT', 'IGT_ALL']:
            pi_0, _ = self.GRASP_NEH_heuristic(show_result=False, save_result=False, random_seed=random_seed)
        else:
            raise NameError('IG_algorithm variant parameter Error')
        pi_0_goal = self.forward_scheduling_approach(pi_0)[self.goal]
        pi_best = pi_0.copy()
        pi_best_goal = self.forward_scheduling_approach(pi_best)[self.goal]
        start_time = time.time()
        while time.time() - start_time < self.time_limit:
            print('IG Iteration')
            pi_1, pi_R = self.destruction(pi_0.copy(), d_S)
            if variant == 'IGT_ALL':
                pi_1 = self.first_improvement_insertion_local_search(pi_1)
            pi_2 = self.construction(pi_1.copy(), pi_R)
            if variant in ['IG_RS', 'IG_GR']:
                pi_3 = self.first_improvement_insertion_local_search(pi_2.copy())
                pi_3_goal = self.forward_scheduling_approach(pi_3)[self.goal]
            elif variant in ['IGT', 'IGT_ALL']:
                pi_3 = self.r_local_search(algorithm='RIS' if r < jP else 'RSS', pi=pi_2.copy(), pi_best=pi_best)
                pi_3_goal = self.forward_scheduling_approach(pi_3)[self.goal]
            else:
                raise NameError('IG algorithm variant parameter error!')
            if pi_3_goal < pi_0_goal:
                pi_0 = pi_3.copy()
                pi_0_goal = self.forward_scheduling_approach(pi_0)[self.goal]
                if pi_3_goal < pi_best_goal:
                    pi_best = pi_3.copy()
                    pi_best_goal = self.forward_scheduling_approach(pi_best)[self.goal]
            elif r < math.exp(-(pi_3_goal - pi_0_goal) / T):
                pi_0 = pi_3.copy()
        best_solution_result = self.forward_scheduling_approach(pi_best)
        self.draw_fs_result_gant_chart(pi_best, method=variant, show_chart=show_result, save_result=save_result)
        return pi_best, best_solution_result

    def VBIH(self, b_max=8, tP=0.5, jP=0.4, random_seed=None, show_result=True, save_result=False):
        """
        Variable block insertion heuristic proposed by (Tasgetiren et al., 2016a, Tasgetiren et al., 2016b,
        Tasgetiren Q. Pan et al., 2017). Unlike the standard block moves, the block size b changes in VBIH,
        allowing for several block moves with different block sizes.

        :param b_max: maxsize for block
        :param tP: τP is a parameter to be adjusted and is used to calculated the constant temperature T. According to
                DOE tP is set to 0.5 for VBIH.
        :param jP: jump probability. According to DOE tP is set to 0.4 for VBIH.
        :param random_seed: random seed
        :param show_result: show result
        :param save_result: save result
        :return: best solution and its forward scheduling result
        """
        random.seed(random_seed)
        r = random.uniform(0, 1)
        random.seed(None)
        T = sum([sum(self.p_kj[k]) for k in range(self.n_stages)]) / (10 * self.n_jobs * self.n_stages) * tP
        pi_0, _ = self.GRASP_NEH_heuristic(show_result=False, save_result=False, random_seed=random_seed)
        pi_0_goal = self.forward_scheduling_approach(pi_0)[self.goal]
        pi_best = pi_0.copy()
        pi_best_goal = self.forward_scheduling_approach(pi_best)[self.goal]
        b_min = 2
        start_time = time.time()
        while time.time() - start_time < self.time_limit:
            print('VBIH Iteration')
            assert b_min < b_max  # there is no do in Python
            b = b_min
            while b <= b_max:
                pi_1, pi_b = self.remove_block(pi_0.copy(), b, random_seed=random_seed)
                pi_2 = self.first_improvement_insertion_local_search(pi_1)
                all_fitness = []
                all_new_pi = []
                for j in range(len(pi_2) + 1):
                    new_pi = pi_2[:j] + pi_b + pi_2[j:]
                    all_new_pi.append(new_pi)
                    all_fitness.append(self.forward_scheduling_approach(new_pi)[self.goal])
                pi_3 = all_new_pi[all_fitness.index(min(all_fitness))]
                pi_4 = self.r_local_search(algorithm='RIS' if r < jP else 'RSS', pi=pi_3.copy(), pi_best=pi_best)
                pi_4_goal = self.forward_scheduling_approach(pi_4)[self.goal]
                if pi_4_goal < pi_0_goal:
                    pi_0 = pi_4.copy()
                    pi_0_goal = self.forward_scheduling_approach(pi_0)[self.goal]
                    if pi_4_goal < pi_best_goal:
                        pi_best = pi_4.copy()
                        pi_best_goal = self.forward_scheduling_approach(pi_best)[self.goal]
                else:
                    b += 1
                    if r < math.exp(-(pi_4_goal - pi_0_goal) / T):
                        pi_0 = pi_4.copy()
        best_solution_result = self.forward_scheduling_approach(pi_best)
        self.draw_fs_result_gant_chart(pi_best, method='VBIH', show_chart=show_result, save_result=save_result)
        return pi_best, best_solution_result


if __name__ == '__main__':
    hfsp_instances = [HFSPInstance(default=True), HFSPInstance(n_jobs=6, n_stages=3, random_seed=1), HFSPInstance(
        n_jobs=10, n_stages=4, random_seed=1)]
    for g in ['TFT', 'C_max']:
        for hfsp_instance in hfsp_instances:
            ma_solver = MetaheuristicAlgorithmSolver(hfsp_instance, goal=g)
            neh_solution_result = ma_solver.NEH_heuristic(show_result=False, save_result=True)
            grasp_neh_solution_result = ma_solver.GRASP_NEH_heuristic(show_result=False, save_result=True, random_seed=1)
            for ig in ['IG_RS', 'IG_GR', 'IGT', 'IGT_ALL']:
                ig_rs_solution_result = ma_solver.IG_algorithm(variant=ig, random_seed=1, show_result=False,
                                                               save_result=True)
            vbih_solution_result = ma_solver.VBIH(show_result=False, random_seed=1, save_result=True)
