from Code.LinearSystem import LinearSystem
from Code.PidController import PidController as PidCtrl
import numpy as np
import matplotlib.pyplot as plt


if __name__ == '__main__':
    applied_voltage = 1.  # Voltage applied to the linear system, relative to the equilibrium voltage, in volts
    ball_position = 0.1  # Starting position of the ball relative to the equilibrium point, in metres
    set_point = 0.  # Set point for the ball to tend to, relative to the equilibrium point, in metres

    # Declare the global variables
    t_final = 1  # The final time of the simulation
    t_sampling = 0.001  # Time between consecutive samples, 30 ms
    ticks = int(t_final / t_sampling)  # Total number of samples taken
    t_span = t_sampling * np.arange(ticks + 1)  # All values of time which were used for sampling

    # Declare the array to store the x_1_bar cache and PID controllers
    x_cache = []
    pid_controllers = [PidCtrl(kp=2, kd=1.9, ki=250, ts=t_sampling)]

    for i in range(len(pid_controllers)):
        # Simulation of the ball with each PID controller
        ball = LinearSystem(x_1_bar=ball_position, v_bar=applied_voltage)  # Create a new ball
        x_cache.append(np.array([ball.get_x_1_bar()]))  # Create a new x cache
        for t in range(ticks):
            voltage = pid_controllers[i].control(ball.get_x_1_bar(), set_point)
            ball.move(voltage, t_sampling, 0)
            x_cache[i] = np.vstack((x_cache[i], [ball.get_x_1_bar()]))

        # Plot all of the x trajectories of the ball simulations on one graph
        label = "PID Controller " + str(i+1)
        plt.plot(t_span, x_cache[i], label=label)
    plt.grid()
    plt.legend()
    plt.xlabel('Time (s)')
    plt.ylabel('$\overline{x}_1$ (m)')
    plt.savefig('.\\Figures\\pid_controlled_system.svg')  # Save the graph as a .svg file
    plt.show()
    print("Done!")
