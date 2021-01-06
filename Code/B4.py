from Code.LinearSystem import LinearSystem
import numpy as np
import matplotlib.pyplot as plt
import control as ctrl

if __name__ == '__main__':
    ball = LinearSystem()  # Create a linear system
    G_x = ball.transfer_function()  # Transfer function of the linear system
    # G_x = DN / (s^3 + (H + P)s^2 + (HP - F)s - FP)

    # Graph for the Bode plot
    f = np.logspace(-1, 3, 1000)
    w = 2 * np.pi * f
    bode_plot = ctrl.bode(G_x, w, dB=True, Hz=True, deg=True)  # Produce the bode plot of G_x against w
    plt.savefig('.\\Figures\\bode_plot.svg')  # Save the graph as an .svg
    plt.show()  # Display the bode plot
