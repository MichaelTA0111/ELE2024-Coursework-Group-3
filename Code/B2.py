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

    # Simulating the linear and non-linear systems at equilibrium
    ball_linear_equil = LinearSystem()  # Create a linear system at equilibrium
    ball_nonlinear_equil = NonlinearSystem(states)  # Create a non-linear system at equilibrium

    linear_trajectory_equil = ball_linear_equil.move()  # Apply v_e for time 0 <= t <= 1, i.e. v_bar = 0
    nonlinear_trajectory_equil = ball_nonlinear_equil.move(v_e)  # Apply v_e for all time 0 <= t <= 1

    # Plot graphs of x position against time with initial conditions at equilibrium
    ball_linear_equil.plotter(linear_trajectory_equil.t,
                              linear_trajectory_equil.y[0].T,
                              title='Linear System Starting at Equilibrium',
                              file_path='.\\Figures\\linear_system_equil.svg')
    ball_nonlinear_equil.plotter(nonlinear_trajectory_equil.t,
                                 nonlinear_trajectory_equil.y[0].T,
                                 title='Non-Linear System Starting at Equilibrium',
                                 file_path='.\\Figures\\nonlinear_system_equil.svg')

    # Simulating the linear and non-linear systems 3.5 cm from the equilibrium position
    states['x_1'] = x_1_e + 0.035
    ball_linear = LinearSystem(x_1_bar=0.035)  # Create a linear system 3.5 cm from equilibrium
    ball_nonlinear = NonlinearSystem(states)  # Create a non-linear system 3.5 cm from equilibrium

    linear_trajectory = ball_linear.move()  # Apply V^e for all time t >= 0, i.e. v_bar = 0
    nonlinear_trajectory = ball_nonlinear.move(v_e)  # Apply V^e for all time t >= 0

    # Plot graphs of x position against time with initial x position 3.5 cm away from equilibrium
    ball_linear.plotter(linear_trajectory.t,
                        linear_trajectory.y[0].T,
                        title='Linear System Starting 3.5cm from the Equilibrium',
                        file_path='.\\Figures\\linear_system_3_5_cm.svg')
    ball_nonlinear.plotter(nonlinear_trajectory.t,
                           nonlinear_trajectory.y[0].T,
                           title='Non-Linear System Starting 3.5cm from the Equilibrium',
                           file_path='.\\Figures\\nonlinear_system_3_5_cm.svg')
