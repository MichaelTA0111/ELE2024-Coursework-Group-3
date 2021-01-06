from Code.LinearSystem import LinearSystem
from Code.PidController import PidController as PidCtrl
from control import TransferFunction as Tf
from control import impulse_response as ir
from control import feedback as fb
import numpy as np


if __name__ == '__main__':
    # Declare time variables
    t_sampling = 0.001  # Time between the consecutive samples in seconds
    dt = 1  # Time for the simulation of the system in seconds
    num_points = 1001  # Resolution of the graph
    t_span = np.linspace(0, dt, num_points)  # All values of time which were used for sampling

    # Declare all systems
    ball = LinearSystem()  # Create a linear system
    pid = PidCtrl(kp=0.125, kd=0.00001, ki=0., ts=t_sampling)  # PID controller
    laser_t_sampling = 0.03  # Sampling time of the laser measurement system

    # Declare all transfer functions
    G_x = ball.transfer_function()  # Transfer function of the linear system
    G_pid = pid.transfer_function()  # Transfer function of the PID controller
    G_laser = Tf([1], [laser_t_sampling, 1])  # Transfer function of the laser measurement system
    G_system = fb(G_x * G_pid, G_laser)  # Transfer function of the whole system

    # Impulse response of the system
    t_imp, system_imp = ir(G_system, T=t_span)

    # Plot a graph for the impulse response
    ball.plotter(t_imp,
                 system_imp,
                 file_path='.\\Figures\\system_impulse_response.svg')  # x position of the ball against time
