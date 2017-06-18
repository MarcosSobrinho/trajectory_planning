import classes
import numpy as np
from scipy.optimize import minimize
from matplotlib import pyplot as plt

'Enter desired values'

v_max = 40
a_max = 10
j_max = 10

v_des = 20
vel_init = 10
acc_init = 0

'Enter lists of obstacles [t_a, t_b, s_a, s_b] here'
obstacle_list = [[4, 5, 30, 50], [8, 9, 80, 90]]

wv = 10
wa = 5
wj = 2

T = 10
N = 30

'Computation of cost, constraints and bounds'

path = classes.CostFunction(T=T, N=N, v_des=v_des, wv=wv, wa=wa, wj=wj)
Constraints = classes.PhysicalConstraints(T=T, N=N, v_max=v_max, a_max=a_max, j_max=j_max)
Bounds = classes.ObstacleConstraints(N=N, T=T, vel_init=vel_init, acc_init=acc_init)

constraints = ({'type': 'ineq', 'fun': Constraints.a_cons}, {'type': 'ineq', 'fun': Constraints.j_cons},
               {'type': 'ineq', 'fun': Constraints.v_cons}, {'type': 'ineq', 'fun': Constraints.v_pos_cons})

bounds = Bounds.obstacles(obstacle_list)

M = len(bounds)
trajectory_list = np.zeros((M, N))
cost_list = np.zeros(M)
x_0 = np.arange(30)**2

for i in range(M):
    res = minimize(path.cost, x_0, method='SLSQP', bounds=bounds[i],
                   constraints=constraints, options={'disp': True})

    trajectory_list[i] = res.x
    cost_list[i] = res.fun

optimal_trajectory_label = np.argmin(cost_list)
optimal_trajectory = trajectory_list[optimal_trajectory_label]

print('The optimal trajectory is:')
print(optimal_trajectory_label + 1)

'From here on it is only code concerning plots'

plot_tools = classes.Plots(T=T, N=N)
x_axis = plot_tools.x_axis()

for obstacle in obstacle_list:
    plot_tools.crossing_obstacles(*obstacle)

for i, trajectory in enumerate(trajectory_list):
    plt.plot(x_axis, trajectory, label=i+1)

plt.grid()
plt.legend()
plt.show()