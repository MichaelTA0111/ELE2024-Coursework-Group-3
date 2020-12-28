from Code.LinearSystem import LinearSystem
import numpy as np
import control as ctrl
import matplotlib.pyplot as plt


if __name__ == '__main__':
    dt = 1
    num_points = 1001
    ball = LinearSystem()
    ball_trajectory = ball.move(0)

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

    plt.plot(t_imp, ball_imp)
    plt.xlabel('Time (s)')
    plt.ylabel('$\overline{x}_1$ (m)')
    plt.grid()
    plt.savefig('.\\Figures\\impulse_response.svg', format='svg')  # Save the graph as a .svg file
    plt.show()

    plt.plot(t_step, ball_step)
    plt.xlabel('Time (s)')
    plt.ylabel('$\overline{x}_1$ (m)')
    plt.grid()
    plt.savefig('.\\Figures\\step_response.svg', format='svg')  # Save the graph as a .svg file
    plt.show()
