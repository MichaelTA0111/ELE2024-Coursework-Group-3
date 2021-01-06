from Code.LinearSystem import LinearSystem
from Code.NonlinearSystem import NonlinearSystem
import numpy as np


if __name__ == '__main__':
    # Dictionary containing values for the Linear and Non-Linear System
    attributes = {
        'mass': 0.425,
        'gravity': 9.81,
        'phi': np.deg2rad(42.0),
        'c_const': 6.815,
        'delta': 0.65,
        'k_spring': 1880,
        'd_length': 0.42,
        'b_damper': 10.4,
        'ell_0': 0.12,
        'ell_1': 0.025,
        'alpha': 1.2,
        'resistance': 53,
    }

    # Equilibrium Values of x_1, x_2, current, voltage
    x_1_e = 0.75 * (attributes['d_length'] + (attributes['mass'] * attributes['gravity'] * np.sin(attributes['phi']) /
                                              attributes['k_spring'])) + 0.25 * attributes['delta']
    x_2_e = 0.
    i_e = np.sqrt(((attributes['mass'] * attributes['gravity'] * np.sin(attributes['phi']))
                   - (attributes['k_spring'] * (x_1_e - attributes['d_length']))) /
                  (-1 * attributes['c_const']) * ((attributes['delta'] - x_1_e) ** 2))
    v_e = i_e * attributes["resistance"]

    # Dictionary containing initial values for the Linear and Non-Linear System
    states = {
        'x_1': x_1_e,
        'x_2': x_2_e,
        'i': i_e
    }

    # Simulating the Linear and Non-Linear System's at Equilibrium
    ball_linear = LinearSystem()  # Creating Linear System
    ball_nonlinear = NonlinearSystem(states, attributes)  # Creating Non-Linear System

    linear_trajectory = ball_linear.move()  # Apply V^e for all time t >= 0, i.e. v_bar = 0
    non_linear_trajectory = ball_nonlinear.move(v_e)  # Apply V^e for all time t >= 0

    # Plot the graph - x position against time - at equilibrium
    ball_linear.plotter(linear_trajectory.t,
                        linear_trajectory.y[0].T,
                        '.\\Figures\\linear_system_equil.svg',
                        'Linear System Starting at Equilibrium')
    ball_nonlinear.plotter(non_linear_trajectory.t,
                           non_linear_trajectory.y[0].T,
                           '.\\Figures\\nonlinear_system_equil.svg',
                           'Non-Linear System Starting at Equilibrium')

    # Simulating the Linear and Non-Linear System's 3.5cm from the Equilibrium Poistion
    states['x_1'] = x_1_e + 0.035
    ball_linear = LinearSystem(x_1_bar=0.035)  # Creating Linear System
    ball_nonlinear = NonlinearSystem(states, attributes)  # Creating Non-Linear System

    linear_trajectory = ball_linear.move()  # Apply V^e for all time t >= 0, i.e. v_bar = 0
    non_linear_trajectory = ball_nonlinear.move(v_e)  # Apply V^e for all time t >= 0

    # Plot the graph - x position against time - 0.035m from equilibrium
    ball_linear.plotter(linear_trajectory.t,
                        linear_trajectory.y[0].T,
                        '.\\Figures\\linear_system_3_5_cm.svg',
                        'Linear System Starting 3.5cm from the Equilibrium')
    ball_nonlinear.plotter(non_linear_trajectory.t,
                           non_linear_trajectory.y[0].T,
                           '.\\Figures\\nonlinear_system_3_5_cm.svg',
                           'Non-Linear System Starting 3.5cm from the Equilibrium')
