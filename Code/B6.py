from Code.LinearSystem import LinearSystem
from Code.PidController import PidController as PidCtrl
from control import TransferFunction as Tf
from control import impulse_response as ir
from control import step_response as sr
from control import feedback as fb
import numpy as np
import sympy as sym


# https://github.com/alchemyst/Dynamics-and-Control/blob/master/tbcontrol/symbolic.py
def routh(p):
    """ Construct the Routh-Hurwitz array given a polynomial in s
    Input: p - a sympy.Poly object
    Output: The Routh-Hurwitz array as a sympy.Matrix object
    """
    coefficients = p.all_coeffs()
    N = len(coefficients)
    M = sym.zeros(N, (N + 1) // 2 + 1)

    r1 = coefficients[0::2]
    r2 = coefficients[1::2]
    M[0, :len(r1)] = [r1]
    M[1, :len(r2)] = [r2]
    for i in range(2, N):
        for j in range(N // 2):
            S = M[[i - 2, i - 1], [0, j + 1]]
            M[i, j] = sym.simplify(-S.det() / M[i - 1, 0])
    sym.simplify(M)
    return M[:, :-1]


def part1():
    """
    This Function is used to generate LaTex for B6.
        It produces a system transfer function in terms of kp,
        kd, and ki of PID controller.
        It runs the denominator of the transfer function through the
        Routh-Hurwitz Tabulation Method.
    :return: nothing
    """
    k_p, k_d, k_i, T = sym.symbols('k_p, k_d, k_i, T', real=True, positive=True, nonzero=True)
    # k, T = sym.symbols('k, T', real=True, positive=True)
    s = sym.symbols("s")

    G_pid = (k_d * s ** 2 + k_p * s + k_i) / s
    # G_pid = (k * s**2 + k * s + k) / s
    G_laser = (1 / ((30 * 10 ** (-3)) * s + 1))
    # G_laser =(1/((T) * s + 1))
    G_x = (8821 / (s ** 3 + 418.4 * s ** 2 + s * 1.768 * 10 ** 4 + 9.28 * 10 ** 5))

    # Determining System TF Symbolically, in terms of kp, kd, and ki
    G_system = (G_pid * G_x) / (1 + G_pid * G_x * G_laser)

    G_system = sym.simplify(G_system)
    G_system = sym.expand(G_system)
    G_system = sym.simplify(G_system)
    G_system = sym.collect(G_system, s)
    print(sym.latex(G_system))
    # print(G_system)

    tf_denom = sym.Poly(sym.fraction(G_system)[1], s)  # Denominator of the Transfer Function
    # print(sym.latex(tf_denom))
    routhTabu = routh(tf_denom)  # Routh-Hurwitz Tabulation
    routhLaTeX = sym.latex(routhTabu).replace("\\\\", "\\\\\n\t")  # LaTeX pastable code
    # print(routhLaTeX)


def part2(kp, kd, ki):
    """
    Checks whether the PID values produces a BIBO stable system
    :param kp: p constant of PID controller
    :param kd: d constant of PID controller
    :param ki: i constant of PID controller
    :return: True if BIBO Stable, else False
    """
    k_p, k_d, k_i = sym.symbols('k_p, k_d, k_i', real=True, positive=True, nonzero=True)
    s = sym.symbols("s")
    G_system = (264.63 * k_d * s ** 3 + 8821 * k_i + s ** 2 * (8821 * k_d + 264.63 * k_p) + s * (
                264.63 * k_i + 8821 * k_p)) / (
                            8821 * k_i + 0.03 * s ** 5 + 13.552 * s ** 4 + 948.8 * s ** 3 + s ** 2 * (
                                8821 * k_d + 45520.0) + s * (8821 * k_p + 928000.0))

    # Substitution
    numerical_tf = G_system.subs(k_p, kp)
    numerical_tf = numerical_tf.subs(k_d, kd)
    numerical_tf = numerical_tf.subs(k_i, ki)

    tf_denom = sym.Poly(sym.fraction(numerical_tf)[1], s)  # Denominator of the Transfer Function
    routhTabu = routh(tf_denom)  # Routh-Hurwitz Tabulation

    col = np.array(routhTabu.col(0))  # Get the first column of the Tabulation

    counter = 0  # Number of Positive Numbers in Tabulation

    for x in range(len(col)):
        print(col[x][0])
        if col[x][0] > 0:
            counter += 1

    if counter == len(col):
        return True
    else:
        return False


if __name__ == '__main__':
    # part1()
    # print(part2(kp=.00125, kd=0.00008, ki=0.001))

    # Declare time variables
    pid_t_sampling = 0.001  # Time between the consecutive samples in seconds
    dt = 1  # Time for the simulation of the system in seconds
    num_points = 1001  # Resolution of the graph
    t_span = np.linspace(0, dt, num_points)  # All values of time which were used for sampling

    # Declare all systems
    ball = LinearSystem()  # Create a linear system
    pid = PidCtrl(kp=0.0001, kd=0.000001, ki=0., ts=pid_t_sampling)  # PID controller
    laser_t_sampling = 0.03  # Sampling time of the laser measurement system

    # Declare all transfer functions
    G_x = ball.transfer_function()  # Transfer function of the linear system
    G_pid = pid.transfer_function()  # Transfer function of the PID controller
    G_laser = Tf([1], [laser_t_sampling, 1])  # Transfer function of the laser measurement system
    G_system = fb(G_x * G_pid, G_laser)  # Transfer function of the whole system

    # Impulse response of the system
    t_imp, system_imp = ir(G_system, T=t_span)

    # Step response of the system
    t_step, system_step = sr(G_system, T=t_span)

    x_axis = [t_imp, t_step]
    y_axis = [system_imp, system_step]
    labels = ['Impulse Response', 'Step Response']
    h_lines = [[0.001, 0.3, '±1 mm'], [-0.001, 0.3, None]]

    ball.plotter(x_axis,
                 y_axis,
                 file_path='.\\Figures\\system_responses.svg',
                 multiplot=True,
                 labels=labels,
                 h_lines=h_lines)
