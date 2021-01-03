from Code.LinearSystem import LinearSystem
import numpy as np
import control as ctrl


if __name__ == '__main__':
    # Declare time variables
    dt = 1  # Time for the simulation of the system
    num_points = 1001  # Resolution
    ball = LinearSystem()
    ball.move()

    # Transfer function
    G_x = ctrl.TransferFunction([ball.get_d() * ball.get_n()],
                                [1,
                                 (ball.get_h() + ball.get_p()),
                                 (ball.get_h() * ball.get_p() - ball.get_f()),
                                 -(ball.get_f() * ball.get_p())])
    # G_x = DN / (s^3 + (H + P)s^2 + (HP - F)s - FP)

    # Impulse and step response of the system
    t_imp, ball_imp = ctrl.impulse_response(G_x, T=np.linspace(0, dt, num_points))
    t_step, ball_step = ctrl.step_response(G_x, T=np.linspace(0, dt, num_points))

    # Plot graphs for impulse and step response
    ball.plotter(t_imp, ball_imp, '.\\Figures\\impulse_response.svg')  # Impulse, x position of the ball against time
    ball.plotter(t_step, ball_step, '.\\Figures\\step_response.svg')  # Step, x position of the ball against time
