import click
import numpy as np
import pickle

from baselines import logger
from baselines.common import set_global_seeds
# import baselines.her.experiment.config as config
import ur_gazebo_test2.experiment.config as config
from baselines.her.rollout import RolloutWorker
import rospy
from modman_comm.msg import modman_state

M_state = None
def modman_state_callback(data):
    global M_state
    M_state = data


@click.command()
@click.argument('policy_file', type=str)
@click.option('--seed', type=int, default=0)
@click.option('--n_test_rollouts', type=int, default=20)
@click.option('--render', type=int, default=1)
def main(policy_file, seed, n_test_rollouts, render):
    set_global_seeds(seed)

    global M_state
    # Load policy.
    with open(policy_file, 'rb') as f:
        policy = pickle.load(f)
    # env_name = policy.info['env_name']
    env_name = 'UR5Slide-v2'

    # init ros node
    rospy.init_node('UR5_slide_play')

    rospy.Subscriber("/drl_state", modman_state, modman_state_callback)

    # Prepare params.
    params = config.DEFAULT_PARAMS
    if env_name in config.DEFAULT_ENV_PARAMS:
        params.update(config.DEFAULT_ENV_PARAMS[env_name])  # merge env-specific parameters in
    params['env_name'] = env_name
    params = config.prepare_params(params)
    config.log_params(params, logger=logger)

    dims = config.configure_dims(params)

    eval_params = {
        'exploit': True,
        'use_target_net': params['test_with_polyak'],
        'compute_Q': True,
        'rollout_batch_size': 1,
        'render': bool(render),
    }

    for name in ['T', 'gamma', 'noise_eps', 'random_eps']:
        eval_params[name] = params[name]
    
    evaluator = RolloutWorker(params['make_env'], policy, dims, logger, **eval_params)
    evaluator.seed(seed)

    # Run evaluation.
    evaluator.clear_history()
    for _ in range(n_test_rollouts):
        key_input = input("CONTINUE TO PLAY...")
        while M_state.state != 2:
            # rospy.sleep(rospy.Duration(0.1))
            print("waiting visual info...")
        evaluator.generate_rollouts()


    # record logs
    for key, val in evaluator.logs('test'):
        logger.record_tabular(key, np.mean(val))
    logger.dump_tabular()


if __name__ == '__main__':
    main()
