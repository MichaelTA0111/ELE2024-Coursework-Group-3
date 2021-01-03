from Code.LinearSystem import LinearSystem
from Code.PidController import PidController as PidCtrl
import control as ctrl
from control import TransferFunction as Tf
import numpy as np


if __name__ == '__main__':
    t_sampling = 0.01
    dt = 1
    num_points = 1001
    t_span = np.linspace(0, dt, num_points)

    ball = LinearSystem()
    ball_tf = ball.transfer_function()
    pid = PidCtrl(kp=0.125, kd=0.00001, ki=25, ts=t_sampling)
    pid_tf = pid.transfer_function()

    laser_t_sampling = 0.03
    laser_tf = Tf([1], [0.03, 1])

    # Feedback TF
    system_tf = ctrl.feedback(ball_tf * pid_tf, laser_tf)

    # Kick the system
    t_imp, system_imp = ctrl.impulse_response(system_tf, T=t_span)

    ball.plotter(t_imp, system_imp, '.\\Figures\\system_impulse_response.svg')
