from Code.LinearSystem import LinearSystem
from Code.PidController import PidController as PidCtrl
import numpy as np
import matplotlib.pyplot as plt


if __name__ == '__main__':
    applied_voltage = 5.  # Input voltage (in volts) applied to the linear system, relative to the equilibrium voltage
    ball_position = 0.1  # Initial position (in metres) of the ball relative to the equilibrium point
    set_point = 0.  # Set point (in metres) for the ball to tend to, relative to the equilibrium point

    # Declare time variables
    t_final = 1  # Time (in seconds) for the simulation of the system
    t_sampling = 0.001  # Time (in seconds) between the consecutive samples
    ticks = int(t_final / t_sampling)  # Total number of samples taken
    t_span = t_sampling * np.arange(ticks + 1)  # All values of time which were used for sampling

    # Declare the array to store the x_1_bar cache and PID controllers
    x_cache = []
    pid_controllers = [PidCtrl(kp=2, kd=1.9, ki=250, ts=t_sampling)]

    for i in range(len(pid_controllers)):
        # Simulation of the ball with each controller i.e. P, Pd or Pid
        ball = LinearSystem(x_1_bar=ball_position, v_bar=applied_voltage)  # Create a new ball
        x_cache.append(np.array([ball.get_x_1_bar()]))  # Create a new x cache
        for t in range(ticks):
            voltage = pid_controllers[i].control(ball.get_x_1_bar(), set_point)
            ball.move(voltage, t_sampling, 0)
            x_cache[i] = np.vstack((x_cache[i], [ball.get_x_1_bar()]))

        # Plot all of the x trajectories of the ball simulations on one graph
        label = "PID Controller " + str(i+1)
        plt.plot(t_span, x_cache[i], label=label)  # Plots the t_span and x_cache values on a graph
    plt.grid()  # Produce a grid for the graph
    plt.legend()  # Produce a key for the graph
    plt.xlabel('Time (s)')
    plt.ylabel('$\overline{x}_1$ (m)')
    plt.savefig('.\\Figures\\pid_controlled_system.svg')  # Save the graph as a .svg file
    plt.show()  # Display the graph
    print("Done!")
