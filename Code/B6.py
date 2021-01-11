from Code.LinearSystem import LinearSystem
from Code.PidController import PidController as PidCtrl
from Code.Routh import Routh
from control import TransferFunction as Tf
from control import impulse_response as ir
from control import step_response as sr
from control import feedback as fb
import numpy as np

if __name__ == '__main__':
    # Declare time variables
    dt = 1  # Time for the simulation of the system in seconds
    num_points = 1001  # Resolution of the graph
    t_span = np.linspace(0, dt, num_points)  # All values of time which were used for sampling
    pid_t_sampling = 0.001  # Time between the consecutive samples of the PID controller in seconds
    laser_t_sampling = 0.03  # Time between the consecutive samples of the laser measurement system in seconds

    # Declare the PID control values
    kp = 0.0001
    kd = 0.000001
    ki = 0.000001

    # Check the Routh-Hurwitz tabulation of this system to see if it could be BIBO stable or not
    Routh.check_routh(kp=kp,
                      kd=kd,
                      ki=ki,
                      pid_t_sampling=pid_t_sampling,
                      laser_t_sampling=laser_t_sampling)

    # Declare all systems
    ball = LinearSystem()  # Create a linear system
    pid = PidCtrl(kp=kp, kd=kd, ki=ki, ts=pid_t_sampling)  # PID controller

    # Declare all transfer functions
    g_x = ball.transfer_function()  # Transfer function of the linear system
    g_pid = pid.transfer_function()  # Transfer function of the PID controller
    g_laser = Tf([1], [laser_t_sampling, 1])  # Transfer function of the laser measurement system
    g_system = fb(g_x * g_pid, g_laser)  # Transfer function of the whole system

    # Impulse response of the system
    t_imp, system_imp = ir(g_system, T=t_span)

    # Step response of the system
    t_step, system_step = sr(g_system, T=t_span)

    # Define variables for the graph
    x_axis = [t_imp, t_step]
    y_axis = [system_imp, system_step]
    labels = ['Impulse Response', 'Step Response']
    h_lines = [[0.001, 0.3, '±1 mm'], [-0.001, 0.3, None]]

    # Plot a graph of x_1_bar (m) against time (s)
    ball.plotter(x_axis,
                 y_axis,
                 file_path='.\\Figures\\system_responses.svg',
                 multiplot=True,
                 labels=labels,
                 h_lines=h_lines)
