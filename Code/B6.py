from Code.LinearSystem import LinearSystem
from Code.PidController import PidController as PidCtrl
from control import TransferFunction as Tf
from control import impulse_response as ir
from control import feedback as fb
import numpy as np


if __name__ == '__main__':
    # Declare time variables
    t_sampling = 0.01  # Time (in seconds) between the consecutive samples
    dt = 1  # Time for the simulation of the system
    num_points = 1001  # Resolution
    t_span = np.linspace(0, dt, num_points)  # All values of time which were used for sampling

    # Moving the ball and PID controller
    ball = LinearSystem()
    ball_tf = ball.transfer_function()
    pid = PidCtrl(kp=0.125, kd=0.00001, ki=25, ts=t_sampling)  # PID controller
    pid_tf = pid.transfer_function()

    # Measurements from laser
    laser_t_sampling = 0.03
    laser_tf = Tf([1], [0.03, 1])

    # Feedback of the combined transfer functions
    system_tf = fb(ball_tf * pid_tf, laser_tf)

    # Impulse response of the system
    t_imp, system_imp = ir(system_tf, T=t_span)

    # Plot graph
    ball.plotter(t_imp, system_imp, '.\\Figures\\system_impulse_response.svg')  # x position of the ball against time
