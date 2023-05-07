import numpy as np
from stable_baselines3.common.vec_env import SubprocVecEnv
from stable_baselines3 import PPO
from stable_baselines3.common.policies import ActorCriticCnnPolicy
import optuna
from stable_baselines3.common.env_util import make_vec_env
from youbotCamGymEnv import youbotCamGymEnv


n_cpu = 4


def optimize_ppo2(trial):
    """ Learning hyperparamters we want to optimise"""
    return {
        'n_steps': int(trial.suggest_loguniform('n_steps', 16, 2048)),
        'gamma': trial.suggest_loguniform('gamma', 0.9, 0.9999),
        'learning_rate': trial.suggest_loguniform('learning_rate', 1e-5, 1.),
        'ent_coef': trial.suggest_loguniform('ent_coef', 1e-8, 1e-1),
        'clip_range': trial.suggest_uniform('clip_ range', 0.1, 0.4),
        'n_epochs': int(trial.suggest_loguniform('n_epochs', 1, 48)),
        'gae_lambda': trial.suggest_uniform('gae_lambda', 0.8, 1.)
    }


def optimize_agent(trial):
    """ Train the model and optimise
        Optuna maximises the negative log likelihood, so we
        need to negate the reward here
    """
    model_params = optimize_ppo2(trial)
    env = youbotCamGymEnv(renders=False, isDiscrete=False)


    model = PPO(ActorCriticCnnPolicy, env, verbose=0, **model_params)
    model.learn(10000)

    rewards = []
    n_episodes, reward_sum = 0, 0.0

    obs = env.reset()
    while n_episodes < 4:
        action, _ = model.predict(obs)
        obs, reward, done, _ = env.step(action)
        reward_sum += reward

        if done:
            rewards.append(reward_sum)
            reward_sum = 0.0
            n_episodes += 1
            obs = env.reset()

    last_reward = np.mean(rewards)
    trial.report(-1 * last_reward)

    return -1 * last_reward


if __name__ == '__main__':
    study = optuna.create_study(study_name='youbotCam_optuna', storage='sqlite:///params.db', load_if_exists=True)
    study.optimize(optimize_agent, n_trials=10, n_jobs=4)
