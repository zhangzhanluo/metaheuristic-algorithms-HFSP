"""
    Author: Zhanluo Zhang
    Author E-mail: zhangzhanluo@outlook.com
    Version: v1.0
    Created Date: 20210602
    Description: Basic functions
"""
import random
from matplotlib import pyplot as plt


class HFSPInstance:
    def __init__(self, n_jobs=None, n_stages=None, machine_layout='e', default=False, random_instance=True):
        """
        Generate instance.

        :param n_jobs: number of jobs
        :param n_stages: number of stages
        :param machine_layout: machine layout type: a, b, c, d, e, see Section 6 for more infos
        :param default: the example given in Section 4.
        :return: Processing time, the number of machines in each stage, instance name
        """
        if not random_instance:
            random.seed(2021)
        self.n_jobs = 6 if default else n_jobs
        self.n_stages = 2 if default else n_stages
        self.name = 'default' if default else 'j{}c{}{}'.format(n_jobs, n_stages, machine_layout)
        self.machine_layout = 'default' if default else machine_layout
        if default:  # a given example at the very beginning
            self.processing_time = [[4, 3, 6, 4, 4, 2],
                                    [5, 5, 3, 3, 3, 3]]
            self.stages_machine = [2, 2]
        else:
            self.processing_time = [[round(random.uniform(0, 100), 2) for _ in range(n_jobs)] for _ in range(n_stages)]
            self.stages_machine = self.generate_stages_machine()
        if not random_instance:
            random.seed(None)

    def generate_stages_machine(self):
        stages_machine = [3 for _ in range(self.n_stages)]
        if self.machine_layout == 'a':
            stages_machine[len(stages_machine) // 2] = 1
        elif self.machine_layout == 'b':
            stages_machine[0] = 1
        elif self.machine_layout == 'c':
            stages_machine[len(stages_machine) // 2] = 2
        elif self.machine_layout == 'd':
            pass
        elif self.machine_layout == 'e':
            stages_machine = [random.randint(3, 5) for _ in stages_machine]
        else:
            raise NameError('There is no {} type for machine layout.'.format(self.machine_layout))
        return stages_machine


def draw_gant_chart(job_machine_infos, ax=None, instance_name=None, method=None, goal=None, TFT=None, C_max=None,
                    show_chart=True, save_chart=True, dpi=300):
    """
    Draw gant chart.

    :param job_machine_infos: list of (job_name, machine_name, start_time, duration)
    :param ax: optional.
    :param instance_name: optional, instance name.
    :param method: algorithm
    :param goal: TFT or C_max
    :param TFT: total flow time
    :param C_max: makespan
    :param show_chart: optional, default true
    :param save_chart: optional, default true
    :param dpi: optional, parameter for saving the pic
    :return: Nothing
    """
    machines_name = list(set([job_machine_info[1] for job_machine_info in job_machine_infos]))
    machines_name.sort()
    if ax is None:
        plt.figure()
        ax = plt.gca()
    for block in job_machine_infos:
        y = machines_name.index(block[1])
        rect = ax.barh(y, left=block[2], width=block[3], height=0.6, color='white', edgecolor='black', alpha=0.8)
        ax.text(rect[0].xy[0] + rect[0]._width * 0.5, rect[0].xy[1] + rect[0]._height * 0.5, block[0], ha='center',
                va='center')
    plt.yticks(range(len(machines_name)), machines_name)
    ax.text(1, 0, 'https://github.com/zhangzhanluo/metaheuristic-algorithms-HFSP', ha='right', va='bottom',
            fontsize=5, transform=ax.transAxes)
    ax.set_xlabel('Time')
    ax.set_ylabel('Stage-Machine')

    if instance_name is not None:
        title = 'Gant Chart for {} Instance \n Goal {}, Method {}, TFT {}, C_max {}'.format(instance_name, goal, method,
                                                                                            TFT, C_max)
        ax.set_title(title)
        plt.tight_layout()
        if save_chart:
            save_path = '02_Results/Pics/'
            name = title.replace('\n', ',')
            plt.savefig(save_path + name + '.png', dpi=dpi)
    if show_chart:
        plt.show()


if __name__ == '__main__':
    # test generate_instance()
    hfsp_instance = HFSPInstance(default=True)
    print(hfsp_instance.processing_time)
    hfsp_instance = HFSPInstance(n_jobs=10, n_stages=5, machine_layout='e')
    print(hfsp_instance.processing_time)

    # test draw_gant_chart()
    blocks = [(1, '1-1', 0, 4),
              (1, '2-2', 4, 5),
              (2, '1-2', 0, 3),
              (2, '2-1', 3, 5),
              (3, '1-2', 3, 6),
              (3, '2-2', 9, 3),
              (4, '1-1', 4, 4),
              (4, '2-1', 8, 3),
              (5, '1-1', 8, 4),
              (5, '2-2', 12, 3),
              (6, '1-2', 9, 2),
              (6, '2-1', 11, 3)]
    draw_gant_chart(blocks)
