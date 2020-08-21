# This is the required initiator for environment.
from gym.envs.registration import register

register(
    id='curling-v0',
    entry_point='gym_curling.envs:CurlingEnv',
)
