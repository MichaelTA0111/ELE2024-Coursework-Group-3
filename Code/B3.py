from Code.LinearSystem import LinearSystem
import numpy as np
from control import TransferFunction as Tf
from control import impulse_response as ir
from control import step_response as sr

if __name__ == '__main__':
    # Declare variables for the simulation
    dt = 1  # Time for the simulation of the system in seconds
    num_points = 1001  # Resolution
    ball = LinearSystem()
    ball.move()

    # Transfer function
    G_x = Tf([ball.get_d() * ball.get_n()],
             [1,
              (ball.get_h() + ball.get_p()),
              (ball.get_h() * ball.get_p() - ball.get_f()),
              -(ball.get_f() * ball.get_p())])
    # G_x = DN / (s^3 + (H + P)s^2 + (HP - F)s - FP)

    # Impulse and step response of the system
    t_imp, ball_imp = ir(G_x, T=np.linspace(0, dt, num_points))
    t_step, ball_step = sr(G_x, T=np.linspace(0, dt, num_points))

    # Plot graphs for impulse and step response
    ball.plotter(t_imp,
                 ball_imp,
                 '.\\Figures\\impulse_response.svg')  # Impulse, x position of the ball against time
    ball.plotter(t_step,
                 ball_step,
                 '.\\Figures\\step_response.svg')  # Step, x position of the ball against time
