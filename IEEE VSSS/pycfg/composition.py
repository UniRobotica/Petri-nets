from abstract_module import abstract, measures
import debug
import math_utils
import numpy as np
from typing import List, Tuple
import univector

ball = measures.ball
obstacles = list([(50, 90), (100, 70), (20, 100)])

def composition(r_x: int, r_y: int, ball_pos: Tuple[int, int], obs_pos: List[Tuple[int, int]]) -> List[float]:

    ball_x, ball_y = ball_pos
    d_ball_x, d_ball_y = math_utils.delta_axis(ball_x, ball_y, r_x, r_y)
    theta = univector.phiR(d_ball_x, d_ball_y)
    phi_tuf = univector.phiTuf(theta, d_ball_x, d_ball_y)

    obstacle = univector.closestObstacle(r_x, r_y, obs_pos)
    obs_x, obs_y = obstacle

    robot_obs_x, robot_obs_y = math_utils.delta_axis(obs_x, obs_y, r_x, r_y)
    R = math_utils.norm(robot_obs_x, robot_obs_y)
    robot_obs_dist = math_utils.norm(robot_obs_x, robot_obs_y)
    
    phi_auf = univector.phiAuf(obs_x, obs_y, r_x, r_y, robot_obs_dist)
    phi_composed = univector.phiComposed(phi_tuf, phi_auf, R, obstacle)

    return univector.Nh(phi_composed)

if __name__ == '__main__':
    debug.debug('composition', composition, ball, obstacles)