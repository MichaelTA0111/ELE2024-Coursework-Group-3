from Code.DynamicalSystem import DynamicalSystem
from Code.LinearSystem import LinearSystem
from Code.NonlinearSystem import NonlinearSystem

if __name__ == '__main__':
    system = DynamicalSystem()  # Create a dynamical system

    # Equilibrium values of x_1, x_2, current, and voltage
    x_1_e = system.get_x_1_e()
    x_2_e = system.get_x_2_e()
    i_e = system.get_i_e()
    v_e = system.get_v_e()

    # Dictionary containing initial values for the non-linear system
    states = {
        'x_1': x_1_e,
        'x_2': x_2_e,
        'i': i_e
    }

    # Declare empty arrays
    linear_x_axes = []
    linear_y_axes = []
    nonlinear_x_axes = []
    nonlinear_y_axes = []
    labels = []

    # Simulating the linear and non-linear systems a range of distances from the equilibrium position
    for x in [0., 0.005, 0.01, 0.015, 0.02, 0.025, 0.03, 0.035]:
        states['x_1'] = x_1_e + x
        ball_linear = LinearSystem(x_1_bar=x)  # Create a linear system 3.5 cm from equilibrium
        linear_trajectory = ball_linear.move()  # Apply v_e_array for time 0 <= t <= 1, i.e. v_bar = 0
        linear_x_axes.append(linear_trajectory.t)  # Append to the time array
        linear_y_axes.append(linear_trajectory.y[0].T)  # Append to the x_1_bar array

        ball_nonlinear = NonlinearSystem(states)  # Create a non-linear system 3.5 cm from equilibrium
        nonlinear_trajectory = ball_nonlinear.move(v_e)  # Apply v_e_array for all time 0 <= t <= 1
        nonlinear_x_axes.append(nonlinear_trajectory.t)  # Append to the time array
        nonlinear_y_axes.append(nonlinear_trajectory.y[0].T)  # Append to the x_1 array

        # Create a label for each plot (cm), using one decimal place
        labels.append(str("{:.1f}".format(100 * x)) + ' cm')

    # Plot graphs of x position against time with initial x position 3.5 cm away from equilibrium
    ball_linear.plotter(linear_x_axes,
                        linear_y_axes,
                        file_path='.\\Figures\\linear_system.svg',
                        multiplot=True,
                        labels=labels)
    ball_nonlinear.plotter(nonlinear_x_axes,
                           nonlinear_y_axes,
                           file_path='.\\Figures\\nonlinear_system.svg',
                           multiplot=True,
                           labels=labels)
