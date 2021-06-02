from code_00_generate_processing_time import generate_processing_time


def forward_scheduling_approach(pi, p_kj, I_k):
    """
    In forward scheduling, the jobs are assigned to the most available machine in the first stage with the order of
    initial solution π. Afterwards, in each following stage, the jobs are ordered with regard to their release times
    from the previous stage and assigned to the most available machine in that order. Note that the order of jobs can
    be different in each stage. In this manner, a complete schedule can be found by using only an initial solution π.

    :param pi: solution
    :param p_kj: Processing time of job j  ∈ J at stage k ∈ M
    :param I_k: Set of machines at stage k  ∈ M
    :return: x_jki and s_kj. x_jki -> 1 if job j is processed at machine i at stage k, 0 otherwise;
    s_kj -> Starting time of job j at stage k. Note that in the article it is x_jik instead of x_jki.
    x_jki is better when coding it.
    """
    n_jobs = len(pi)
    n_stages = len(I_k)
    assert len(p_kj) == n_stages and len(p_kj[0]) == n_jobs
    system_clock = 0
    jobs_finishing_time = [0.0 for _ in pi]
    jobs_stage = [-1 for _ in pi]
    machines_status = [[0.0 for _ in range(I_k[i])] for i in range(n_stages)]
    x_jki = [[[0 for _ in range(I_k[k])] for k in range(n_stages)] for _ in range(n_jobs)]
    s_kj = [[-1 for _ in range(n_jobs)] for _ in range(n_stages)]
    while True:
        for j in range(n_jobs):
            # 对所有任务进行遍历，判断是否可以结束以及是否可以开始
            if jobs_finishing_time[j] <= system_clock:  # 任务需要转移到下一阶段
                k = jobs_stage[j]
                if k >= 0:  # 初始状态下没有机器在工作
                    current_machine = x_jki[j][k].index(1)
                    machines_status[k][current_machine] = 0.0  # 任务时间到了，就可以结束了，当前机器占用状态重置，这对应于无限buffer的情况
                if k == n_stages - 1:  # 是倒数第一阶段，则直接标记任务离开系统
                    jobs_finishing_time[j] = 1e10  # 用一个无穷大的数字表示任务已经永远结束
                else:  # 不是最后一个阶段，需要进行下一步开始判断
                    if min(machines_status[k + 1]) == 0.0:  # 下一个阶段有可用机器
                        # 任务转移至下一个阶段，任务的结束时间进行更新
                        jobs_stage[j] = k + 1
                        # 注意！操作时间的矩阵，j的含义与此处j的含义需要用solution解读一下（pi[j]-1）
                        jobs_finishing_time[j] = system_clock + p_kj[k + 1][pi[j]-1]
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
    return x_jki, s_kj


def calculate_cmax_cj(s_kj, p_kj):
    """
    Calculate C_max (Maximum completion time (makespan)) and C_j (Completion time of job j)
    :param s_kj: Starting time of job j at stage k
    :param p_kj: Processing time of job j  ∈ J at stage k ∈ M
    :return: C_max and C_j
    """
    all_c_j = [s_kj[-1][j] + p_kj[-1][j] for j in range(len(s_kj[0]))]
    return max(all_c_j), all_c_j


if __name__ == '__main__':
    processing_time = generate_processing_time(default=True)
    solution = [1, 2, 3, 4, 5, 6]
    stage_machines = [2, 2]
    xjki, skj = forward_scheduling_approach(solution, processing_time, stage_machines)
    cmax, cj = calculate_cmax_cj(skj, processing_time)
    # pi = solution
    # p_kj = processing_time
    # I_k = stage_machines
