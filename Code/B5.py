from Code.LinearSystem import LinearSystem
from Code.PidController import PidController
import numpy as np
import matplotlib.pyplot as plt
import control as ctrl
from control import impulse_response as ir


if __name__ == '__main__':
    ball = LinearSystem()

    # Declare variables for helping to draw the graph
    num_points = 1001  # The resolution of the graph
    dt = 1  # Time t ranges between 0 and 1 seconds

    # Define the PID controller to be used
    kp = 1075
    ki = 1
    kd = 19
    ball_pid = PidController.pid(kp, ki, kd)

    # Use closed loop feedback to combine the PID controller with the System
    tf_G_x = ctrl.feedback(ball.transfer_function(), ball_pid)
    t_imp, x_1_bar_imp = ir(tf_G_x, T=np.linspace(0, dt, num_points))

    # Plot the rod angle against time using the results from G_theta
    plt.plot(t_imp, x_1_bar_imp)
    plt.grid()
    plt.xlabel('Time (s)')
    plt.ylabel('$\overline{x}_1$ (m)')
    plt.savefig('.\\Figures\\b5.svg')  # Save the graph as a .svg file
    plt.show()
