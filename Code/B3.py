from Code.LinearSystem import LinearSystem
import numpy as np
from control import impulse_response as ir
from control import step_response as sr

if __name__ == '__main__':
    dt = 1  # Time for the simulation of the system in seconds
    num_points = 1001  # Resolution of the graph

    ball = LinearSystem()  # Create a linear system
    G_x = ball.transfer_function()  # Transfer function of the linear system
    # G_x = DN / (s^3 + (H + P)s^2 + (HP - F)s - FP)

    # Impulse and step response of the system
    t_imp, ball_imp = ir(G_x, T=np.linspace(0, dt, num_points))
    t_step, ball_step = sr(G_x, T=np.linspace(0, dt, num_points))

    # Plot graphs for the impulse and step responses
    ball.plotter(t_imp,
                 ball_imp,
                 file_path='.\\Figures\\impulse_response.svg')  # Impulse, x position of the ball against time
    ball.plotter(t_step,
                 ball_step,
                 file_path='.\\Figures\\step_response.svg')  # Step, x position of the ball against time
