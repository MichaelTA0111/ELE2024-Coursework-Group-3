from Code.LinearSystem import LinearSystem
from Code.PidController import PidController as PidCtrl
import numpy as np


if __name__ == '__main__':
    applied_voltage = 2.  # Input voltage (volts) applied to the linear system, relative to the equilibrium voltage
    ball_position = 0.1  # Initial position (metres) of the ball relative to the equilibrium point
    set_point = 0.  # Set point (metres) for the ball to tend to, relative to the equilibrium point
    ball = LinearSystem(x_1_bar=ball_position, v_bar=applied_voltage)  # Create a new ball

    # Declare time variables
    t_final = 1  # Time (in seconds) for the simulation of the system
    t_sampling = 0.001  # Time (in seconds) between the consecutive samples
    ticks = int(t_final / t_sampling)  # Total number of samples taken
    t_span = t_sampling * np.arange(ticks + 1)  # All values of time which were used for sampling

    # Declare the array to store the x_1_bar cache
    x_cache = np.array([ball.get_x_1_bar()])
    pid = PidCtrl(kp=2, kd=1.9, ki=500, ts=t_sampling)

    # Simulation of the ball using the PID controller
    for t in range(ticks):
        voltage = pid.control(ball.get_x_1_bar(), set_point)
        ball.move(voltage, t_sampling, 0)
        x_cache = np.vstack((x_cache, [ball.get_x_1_bar()]))

    ball.plotter(t_span,
                 x_cache,
                 '.\\Figures\\pid_controlled_system.svg')
