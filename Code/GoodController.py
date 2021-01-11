from Code.Common.LinearSystem import LinearSystem
from Code.Common.PidController import PidController as PidCtrl
import numpy as np


if __name__ == '__main__':
    applied_voltage = 2.  # Input voltage (volts) applied to the linear system, relative to the equilibrium voltage
    ball_position = 0.1  # Initial position (metres) of the ball relative to the equilibrium point
    set_point = 0.  # Set point (metres) for the ball to tend to, relative to the equilibrium point
    ball = LinearSystem(x_1_bar=ball_position, v_bar=applied_voltage)  # Create a linear system

    # Declare time variables
    t_final = 1  # Time (seconds) for the simulation of the system
    t_sampling = 0.001  # Time (seconds) between the consecutive samples
    ticks = int(t_final / t_sampling)  # Total number of samples taken
    t_span = t_sampling * np.arange(ticks + 1)  # All values of time which were used for sampling

    num_points = 2  # Resolution of the graph, 2 is sufficient here

    x_cache = np.array([ball.get_x_1_bar()])  # Array to store the x_1_bar cache
    pid = PidCtrl(kp=70, kd=5.5, ki=450, ts=t_sampling)  # PID controller

    # Simulation of the ball using the PID controller
    for t in range(ticks):
        voltage = pid.control(ball.get_x_1_bar(), set_point)  # Calculate the PID control variable
        ball.move(voltage, t_sampling, num_points)  # Move the ball
        x_cache = np.vstack((x_cache, [ball.get_x_1_bar()]))  # Append the ball position to the array

    # Plot a graph of x_1_bar (m) against time (s)
    ball.plotter(t_span,
                 x_cache,
                 file_path='.\\Figures\\pid_controlled_system.svg')
