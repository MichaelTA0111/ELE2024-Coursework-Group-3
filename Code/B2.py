from Code.LinearSystem import LinearSystem
from Code.NonlinearSystem import NonlinearSystem
import numpy as np
import matplotlib.pyplot as plt


def plotter(x_axis, y_axis, x_name, y_name, title=None, file_path=None):
    """
    Function that plots a graph
    Parameters
    ----------
    x_axis: For values that are required to be plotted on the x-axis
    y_axis: For values that are required to be plotted on the y-axis
    x_name: Name of x-axis
    y_name: Name of y-axis
    title: Title for the graph

    Returns: Nothing
    -------

    """
    plt.xlabel(x_name)
    plt.ylabel(y_name)
    plt.title(title)
    plt.grid(True, "both", "both")  # Produces a grid on the graph
    plt.plot(x_axis, y_axis)  # Plots the x-axis and y-axis values on a graph

    if file_path:
        save_plot(file_path)


def save_plot(file_path):
    plt.savefig(file_path, format="svg")  # Save the graph
    plt.show()  # Displays the graph


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

# Equilibrium Values of x_1, current, voltage
x_1_e = 0.75 * (attributes['d_length'] + (attributes['mass'] * attributes['gravity'] * np.sin(attributes['phi']) /
                                          attributes['k_spring'])) + 0.25 * attributes['delta']
i_e = np.sqrt(((attributes['mass'] * attributes['gravity'] * np.sin(attributes['phi']))
               - (attributes['k_spring'] * (x_1_e - attributes['d_length']))) /
              (-1 * attributes['c_const']) * ((attributes['delta'] - x_1_e) ** 2))
v_e = i_e * attributes["resistance"]

# Dictionary containing initial values for the Linear and Non-Linear System
states = {
    "x_1": x_1_e,
    "x_2": 0.,
    "i": i_e
}

# Simulating the Linear and Non-Linear System's at Equilibrium
linear = LinearSystem()  # Creating Linear System
non_linear = NonlinearSystem(states, attributes)  # Creating Non-Linear System

linear_trajectory = linear.move(v_e, 1)  # Apply V^e for all time t >= 0
non_linear_trajectory = non_linear.move(v_e, 1)  # Apply V^e for all time t >= 0

# Plot the graph - x position against time - at equilibrium
plotter(linear_trajectory.t, linear_trajectory.y[0].T, "t - time (s)", "$\overline{x}_1$ - position (m)",
        "Linear System at Equilibrium", '.\\Figures\\linear_system_equil.svg')
plotter(non_linear_trajectory.t, non_linear_trajectory.y[0].T, "t - time (s)", "$x_1$ - position (m)",
        "Non-Linear System at Equilibrium", '.\\Figures\\nonlinear_system_equil.svg')

# Simulating the Linear and Non-Linear System's 3.5cm from the Equilibrium Poistion
states["x_1"] = x_1_e + 0.035
linear = LinearSystem(x_1_bar=0.035)  # Creating Linear System
non_linear = NonlinearSystem(states, attributes)  # Creating Non-Linear System

linear_trajectory = linear.move(v_e, 1)  # Apply V^e for all time t >= 0
non_linear_trajectory = non_linear.move(v_e)  # Apply V^e for all time t >= 0

# Plot the graph - x position against time - 0.1m from equilibrium
plotter(linear_trajectory.t, linear_trajectory.y[0].T, "t - time (s)", "$\overline{x}_1$ - position (m)",
        "Linear System starting 3.5cm from Equilibrium", '.\\Figures\\linear_system_3_5_cm.svg')
plotter(non_linear_trajectory.t, non_linear_trajectory.y[0].T, "t - time (s)", "$x_1$ - position (m)",
        "Non-Linear System starting 3.5cm from Equilibrium", '.\\Figures\\nonlinear_system_3_5_cm.svg')
