from Code.LinearSystem import LinearSystem
import numpy as np
import control as ctrl


if __name__ == '__main__':
    dt = 1
    num_points = 1001
    ball = LinearSystem()
    ball.move(0)

    # Transfer function
    G_x = ctrl.TransferFunction([ball.get_d() * ball.get_n()],
                                [1,
                                 (ball.get_h() + ball.get_p()),
                                 (ball.get_h() * ball.get_p() - ball.get_f()),
                                 -(ball.get_f() * ball.get_p())])
    # G_x = DN / (s^3 + (H + P)s^2 + (HP - F)s - FP)

    # Response of the system
    t_imp, ball_imp = ctrl.impulse_response(G_x, T=np.linspace(0, dt, num_points))
    t_step, ball_step = ctrl.step_response(G_x, T=np.linspace(0, dt, num_points))

    ball.plotter(t_imp, ball_imp, '.\\Figures\\impulse_response.svg')
    ball.plotter(t_step, ball_step, '.\\Figures\\step_response.svg')
