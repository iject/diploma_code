import gymnasium as gym

from stable_baselines3 import SAC
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.callbacks import CheckpointCallback, EvalCallback

from gymnasium import ObservationWrapper
from gymnasium.wrappers import RescaleAction

import numpy as np

import glob
import shutil
import os


ENV_ID = "Ant-v5"
ALGO = SAC

N_TIMESTEPS = int(1e6)
N_EVALUATIONS = 10
N_CHECKPOINTS = 10

N_ENVS = 4
EVAL_FREQ = max(int(N_TIMESTEPS / N_EVALUATIONS) // N_ENVS, 1)
SAVE_FREQ = max(int(N_TIMESTEPS / N_CHECKPOINTS) // N_ENVS, 1)

hyperparams = dict(learning_starts = 10_000)

custom_env_kwargs = {
    "xml_file": r"/content/ant_model.xml",
    "include_cfrc_ext_in_observation": True,
    "contact_cost_weight": 5e-4,
    "ctrl_cost_weight": 0.5,
    "healthy_reward": 0.5,
    }

render_env_kwargs = {"render_mode": "human", "width": 1280, "height": 720}

checkpoint_callback_name_prefix = "SAC_custom"
root_dir = r"root"
os.makedirs(root_dir, exist_ok=True)


def create_storage_folders(root_dir=None, create: bool = True) -> list:
  storages_dict = {"monitor_dir": "monitor_storage",
                  "callback_dir": "callback_storage",
                  "tb_dir": "tb_storage",
                   }
  if root_dir is not None:
    storages_dict['monitor_dir'] = os.path.join(root_dir, storages_dict["monitor_dir"])
    storages_dict['callback_dir'] = os.path.join(root_dir, storages_dict["callback_dir"])
    storages_dict['tb_dir'] = os.path.join(root_dir, storages_dict["tb_dir"])

  if create:
    os.makedirs(storages_dict["monitor_dir"], exist_ok=True)
    os.makedirs(storages_dict["callback_dir"], exist_ok=True)
    os.makedirs(storages_dict["tb_dir"], exist_ok=True)

 
  monitors_dict = {"train": os.path.join(storages_dict["monitor_dir"], "train_monitor"),
                   "eval": os.path.join(storages_dict["monitor_dir"], "eval_monitor")}

 
  callbacks_dict = {"eval": os.path.join(storages_dict["callback_dir"], "eval_callback"),
                    "checkpont": os.path.join(storages_dict["callback_dir"], "checkpoint_callback")}

 
  tb_dict = {"experiment": os.path.join(storages_dict["tb_dir"], "experiment_folder")}

  return storages_dict, monitors_dict, callbacks_dict, tb_dict


def delete_storage_folders(storage_folders, verbose = 0):
  if isinstance(storage_folders, str):
    storage_folders = glob.glob("/content/*")

  if verbose == 1: print(storage_folders)

  if input("Введи 'del' для удаления папок: ") != 'del': raise Exception

  for folder in storage_folders:
    shutil.rmtree(folder)

start_position = np.array([ 0.19938886, 0.33333349, -0.19938886, 0.33333349,
                            0.19938886, -0.33333331, -0.19938886, -0.33333331])

class TestWrapper(ObservationWrapper):
    def __init__(self, env):
      super().__init__(env)
      self.__action = start_position
      self.__action_queue = np.full((3,8), self.__action)
      old_obs_shape = self.observation_space.shape
      action_queue_shape = self.__action_queue.shape
      self.observation_space = gym.spaces.Box(low=-np.inf,
                                              high=np.inf,
                                              shape=(old_obs_shape[0] +
                                                      action_queue_shape[0] * action_queue_shape[1],),
                                              dtype=np.float64)

      self._debug_mode = False
      self.act_dtype = env.action_space.dtype

    def _debug_list_init(self):
      self._action_list = []
      self._observation_list = []
      self._reward_list = []
      self._info_list = []
      self._debug_mode = True


    def _append_debug_list(self, observation, action, reward, info):
      self._observation_list.append(observation)
      self._action_list.append(action)
      self._reward_list.append(reward)
      self._info_list.append(info)


    def step(self, action):
        self.__action = action

        observation, reward, terminated, truncated, info = self.env.step(action)
        self.__action_queue[1:] = self.__action_queue[0:-1]
        self.__action_queue[0] = action
        observation = self.observation(observation)

        if self._debug_mode: self._append_debug_list(observation,
                                                     action,
                                                     reward,
                                                     info)

        return observation, reward, terminated, truncated, info


    def reset(self, *, seed = None, options = None):
        self.__action = start_position
        self.__action_queue = np.full((3,8), self.__action)

        observation, info = self.env.reset(seed=seed, options=options)
        observation, reward, terminated, truncated, info = self.step(self.__action)
       

        if self._debug_mode: self._append_debug_list(observation,
                                                     self.__action,
                                                     None,
                                                     info)

        return observation, info


    def observation(self, obs):
        return np.concatenate((obs, self.__action_queue.flatten()),
                              dtype=np.float64)


def create_env(render_env_kwargs={}, **env_kwargs):
  custom_env = gym.make(ENV_ID, **custom_env_kwargs, **render_env_kwargs, **env_kwargs)
  wrapped_env = RescaleAction(custom_env,
                            min_action=-1, max_action=1)
  test_wrapped = TestWrapper(wrapped_env)

  return test_wrapped


if __name__ == "__main__":
    storages_dict, monitors_dict, callbacks_dict, tb_dict = create_storage_folders(root_dir)

    train_env = make_vec_env(create_env, N_ENVS, monitor_dir=monitors_dict["train"])
    eval_env = make_vec_env(create_env, N_ENVS, monitor_dir=monitors_dict["eval"])

    eval_callback = EvalCallback(eval_env,
                            best_model_save_path=callbacks_dict["eval"],
                            log_path=callbacks_dict["eval"],
                            eval_freq=EVAL_FREQ)


    checkpoint_callback = CheckpointCallback(save_freq=SAVE_FREQ,
                            save_path=callbacks_dict["checkpont"],
                            name_prefix=checkpoint_callback_name_prefix,
                            save_replay_buffer=True,
                            save_vecnormalize=True,)

    model = ALGO("MlpPolicy", train_env, **hyperparams,
                tensorboard_log=tb_dict["experiment"])

    model.learn(N_TIMESTEPS, tb_log_name=f"{ALGO.__name__}_{int(N_TIMESTEPS)}_timesteps",
                callback=[eval_callback, checkpoint_callback],
                progress_bar=True)