from Code.DynamicalSystem import DynamicalSystem
from Code.LinearSystem import LinearSystem
from Code.NonlinearSystem import NonlinearSystem

if __name__ == '__main__':
    system = DynamicalSystem()

    # Equilibrium Values of x_1, x_2, current, voltage
    x_1_e = system.get_x_1_e()
    x_2_e = system.get_x_2_e()
    i_e = system.get_i_e()
    v_e = system.get_v_e()

    # Dictionary containing initial values for the Linear and Non-Linear System
    states = {
        'x_1': x_1_e,
        'x_2': x_2_e,
        'i': i_e
    }

    # Simulating the Linear and Non-Linear System's at Equilibrium
    ball_linear_equil = LinearSystem()  # Creating Linear System
    ball_nonlinear_equil = NonlinearSystem(states)  # Creating Non-Linear System

    linear_trajectory_equil = ball_linear_equil.move()  # Apply V^e for all time t >= 0, i.e. v_bar = 0
    nonlinear_trajectory_equil = ball_nonlinear_equil.move(v_e)  # Apply V^e for all time t >= 0

    # Plot the graph - x position against time - at equilibrium
    ball_linear_equil.plotter(linear_trajectory_equil.t,
                              linear_trajectory_equil.y[0].T,
                              '.\\Figures\\linear_system_equil.svg',
                              'Linear System Starting at Equilibrium')
    ball_nonlinear_equil.plotter(nonlinear_trajectory_equil.t,
                                 nonlinear_trajectory_equil.y[0].T,
                                 '.\\Figures\\nonlinear_system_equil.svg',
                                 'Non-Linear System Starting at Equilibrium')

    # Simulating the Linear and Non-Linear System's 3.5cm from the Equilibrium Position
    states['x_1'] = x_1_e + 0.035
    ball_linear = LinearSystem(x_1_bar=0.035)  # Creating Linear System
    ball_nonlinear = NonlinearSystem(states)  # Creating Non-Linear System

    linear_trajectory = ball_linear.move()  # Apply V^e for all time t >= 0, i.e. v_bar = 0
    nonlinear_trajectory = ball_nonlinear.move(v_e)  # Apply V^e for all time t >= 0

    # Plot the graph - x position against time - 0.035m from equilibrium
    ball_linear.plotter(linear_trajectory.t,
                        linear_trajectory.y[0].T,
                        '.\\Figures\\linear_system_3_5_cm.svg',
                        'Linear System Starting 3.5cm from the Equilibrium')
    ball_nonlinear.plotter(nonlinear_trajectory.t,
                           nonlinear_trajectory.y[0].T,
                           '.\\Figures\\nonlinear_system_3_5_cm.svg',
                           'Non-Linear System Starting 3.5cm from the Equilibrium')
