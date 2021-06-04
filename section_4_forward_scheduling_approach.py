"""
    Author: Zhanluo Zhang
    Author E-mail: zhangzhanluo@outlook.com
    Version: v1.0
    Created Date: 20210602
    Description: Forward scheduling approach and calculate C_max and C_j
"""
from section_0_preparing import HFSPInstance, draw_gant_chart


def forward_scheduling_approach(instance, pi):
    """
    In forward scheduling, the jobs are assigned to the most available machine in the first stage with the order of
    initial solution π. Afterwards, in each following stage, the jobs are ordered with regard to their release times
    from the previous stage and assigned to the most available machine in that order. Note that the order of jobs can
    be different in each stage. In this manner, a complete schedule can be found by using only an initial solution π.

    :param instance: HFSP Instance.
    :param pi: solution
    :return: x_jki, s_kj, c_max, c_j. x_jki -> 1 if job j is processed at machine i at stage k, 0 otherwise;
    s_kj -> Starting time of job j at stage k. Note that in the article it is x_jik instead of x_jki.
    x_jki is better when coding it.
    """
    n_jobs = len(pi)
    n_stages = instance.n_stages
    p_kj = instance.processing_time
    I_k = instance.stages_machine
    system_clock = 0
    jobs_finishing_time = [0.0 for _ in pi]  # 记录任务完成时间
    jobs_stage = [-1 for _ in pi]  # 记录任务阶段
    machines_status = [[0.0 for _ in range(I_k[i])] for i in range(n_stages)]  # 记录机器正在处理的任务完成时间或者空闲
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
                    jobs_finishing_time[j] = 1e6  # 用一个无穷大的数字表示任务已经永远结束
                else:  # 不是最后一个阶段，需要进行下一步开始判断
                    if min(machines_status[k + 1]) == 0.0:  # 下一个阶段有可用机器
                        # 任务转移至下一个阶段，任务的结束时间进行更新
                        jobs_stage[j] = k + 1
                        # 注意！操作时间的矩阵，j的含义与此处j的含义需要用solution解读一下（pi[j]-1）
                        jobs_finishing_time[j] = system_clock + p_kj[k + 1][pi[j] - 1]
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
    all_c_j = [s_kj[-1][j] + p_kj[-1][j] for j in range(len(s_kj[0]))]
    return x_jki, s_kj, max(all_c_j), all_c_j


if __name__ == '__main__':
    hfsp_instances = [HFSPInstance(default=True), HFSPInstance(n_jobs=6, n_stages=3, random_instance=False)]
    for hfsp_instance in hfsp_instances:
        solution = [4, 2, 3, 6]
        xjki, skj, _, _ = forward_scheduling_approach(hfsp_instance, solution)
        job_machine_info = []
        for j in range(len(solution)):
            for k in range(hfsp_instance.n_stages):
                machine_id = xjki[j][k].index(1)
                job_machine_info.append(
                    (solution[j], '{}-{}'.format(k + 1, machine_id + 1), skj[k][j],
                     hfsp_instance.processing_time[k][solution[j] - 1]))
        draw_gant_chart(job_machine_info,
                        title='Gant Chart for {} Instance (forward scheduling approach)'.format(hfsp_instance.name),
                        save_path='02_Results/40_Forward_Scheduling/Pics/')
