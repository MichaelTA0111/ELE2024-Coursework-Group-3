from Code.LinearSystem import LinearSystem
from scipy import signal
import matplotlib.pyplot as plt


if __name__ == '__main__':
    dt = 1
    num_points = 1001
    ball = LinearSystem()
    ball.move(0)

    sys = signal.TransferFunction([ball.get_d() * ball.get_n()],
                                  [1,
                                   (ball.get_h() + ball.get_p()),
                                   (ball.get_h() * ball.get_p() - ball.get_f() ),
                                   (-1 * (ball.get_f() * ball.get_p()))])
    w, mag, phase = signal.bode(sys)

    # Graph for bode plots
    plt.figure()
    plt.semilogx(w, mag)    # Bode magnitude plot
    plt.grid()
    plt.xlabel('ω (Log Scale)')
    plt.ylabel('|G$_x$(jω)|$_{dB}$')
    plt.figure()
    plt.semilogx(w, phase)  # Bode phase plot
    plt.xlabel('ω (Log Scale)')
    plt.ylabel('arg[G$_x$(jω)]')
    plt.show()