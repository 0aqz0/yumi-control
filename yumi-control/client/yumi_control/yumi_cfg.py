from yumi_utils import *
import numpy as np

yumi_leftarm_lower_limits = np.array([-2.94,-2.50,-2.94,-2.15,-5.06,-1.53,-3.99])
yumi_leftarm_upper_limits = np.array([ 2.94, 0.75, 2.94, 1.39, 5.06, 2.40, 3.99])
yumi_rightarm_lower_limits = np.array([-2.94,-2.50,-2.94,-2.15,-5.06,-1.53,-3.99])
yumi_rightarm_upper_limits = np.array([ 2.94, 0.75, 2.94, 1.39, 5.06, 2.40, 3.99])

yumi_leftarm_lower_limits_indegrees = rad2degree( yumi_leftarm_lower_limits)
yumi_leftarm_upper_limits_indegrees = rad2degree( yumi_leftarm_upper_limits)
yumi_rightarm_lower_limits_indegrees =  rad2degree(yumi_rightarm_lower_limits)
yumi_rightarm_upper_limits_indegrees =  rad2degree(yumi_rightarm_upper_limits)