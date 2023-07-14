#!/usr/bin/env python3
from reach import runReachStudy
from reach_ros import init_ros, get_parameter
import sys
from yaml import safe_load


def main():
    init_ros(sys.argv)

    with open(get_parameter('config_file'), 'r') as f:
        config = safe_load(f)

    config_name = get_parameter('config_name')
    results_dir = get_parameter('results_dir')

    print('Starting Python reach study node...')
    runReachStudy(config, config_name, results_dir, True)


if __name__ == '__main__':
    main()
