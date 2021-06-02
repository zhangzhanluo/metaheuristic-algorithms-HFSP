"""
    Author: Zhanluo Zhang
    Author E-mail: zhangzhanluo@outlook.com
    Version: v1.0
    Created Date: 20210602
    Description: provide processing time
"""


def generate_processing_time(n_jobs=None, n_stages=None, default=False):
    if default:
        return [[4, 3, 6, 4, 4, 2],
                [5, 5, 3, 3, 3, 3]]
    else:
        pass


if __name__ == '__main__':
    print(generate_processing_time(default=True))
