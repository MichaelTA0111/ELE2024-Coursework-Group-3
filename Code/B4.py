from Code.LinearSystem import LinearSystem
import numpy as np
import matplotlib.pyplot as plt
from control import TransferFunction as Tf
import control as ctrl

if __name__ == '__main__':
    ball = LinearSystem()

    # Transfer function
    G_x = Tf([ball.get_d() * ball.get_n()],
             [1,
              (ball.get_h() + ball.get_p()),
              (ball.get_h() * ball.get_p() - ball.get_f()),
              -(ball.get_f() * ball.get_p())])
    # G_x = DN / (s^3 + (H + P)s^2 + (HP - F)s - FP)

    # Graph for the Bode plot
    f = np.logspace(-1, 3, 1000)
    w = 2 * np.pi * f
    bode_plot = ctrl.bode(G_x, w, dB=True, Hz=True, deg=True)  # Produce the bode plot of G_x against w
    plt.savefig('.\\Figures\\bode_plot.svg')  # Save the graph as an .svg
    plt.show()  # Display the bode plot
