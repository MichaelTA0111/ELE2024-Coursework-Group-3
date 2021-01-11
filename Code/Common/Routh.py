import numpy as np
import sympy as sym


class Routh:
    @staticmethod
    def routh(p):
        """
        Static method to construct the Routh-Hurwitz array given a polynomial in s
        Code sourced from https://github.com/alchemyst/Dynamics-and-Control/blob/master/tbcontrol/symbolic.py
        :param p: A sympy.Poly object
        :return: The Routh-Hurwitz array as a sympy.Matrix object
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

    @staticmethod
    def printer(g_system, s):
        """
        Static method to print the system transfer function and the Routh-Hurwitz tabulation
        :param g_system: The transfer function of the system
        :param s: The SymPy symbol s
        :return: The Routh-Hurwitz tabulation of g_system
        """
        # Print the transfer function
        print('Transfer Function:')
        print(g_system)
        print('LaTeX Format:')
        print(sym.latex(g_system))

        # Calculate the routh tabulation
        tf_denom = sym.Poly(sym.fraction(g_system)[1], s)  # Denominator of the transfer function
        routh_tab = Routh.routh(tf_denom)  # Routh-Hurwitz tabulation
        routh_latex = sym.latex(routh_tab).replace("\\\\", "\\\\\n\t")  # LaTeX format which can be directly copied
        print('Routh-Hurwitz Tabulation:')
        print(routh_latex)

        return routh_tab

    @staticmethod
    def check_routh(kp, kd, ki, pid_t_sampling, laser_t_sampling):
        """
        Static method to:
            Generate LaTex for B6
            Check whether the PID values produce a BIBO stable system
            Produces a system transfer function in terms of kp, kd, and ki of the PID controller
            Runs the denominator of the transfer function through the Routh-Hurwitz Tabulation Method
        :param kp: P constant of the PID controller
        :param kd: D constant of the PID controller
        :param ki: I constant of the PID controller
        :param pid_t_sampling: Time between the consecutive samples of the PID controller in seconds
        :param laser_t_sampling: Time between the consecutive samples of the laser measurement system in seconds
        :return: True if BIBO Stable, else False
        """
        # Define symbols
        k_p, k_d, k_i, = sym.symbols('k_p, k_d, k_i', real=True, positive=True, nonzero=True)
        s = sym.symbols("s")

        # Define the transfer functions symbolically
        g_pid_sym = (k_d * s ** 2 + k_p * s + k_i) / s
        g_laser_num = (1 / (laser_t_sampling * s + 1))
        g_x_num = (3781 / (s ** 3 + 395.1 * (s ** 2) + 7654 * s + 3.977 * (10 ** 5)))

        # Determine the system transfer function symbolically in terms of kp, kd, and ki
        g_system_sym = (g_pid_sym * g_x_num) / (1 + g_pid_sym * g_x_num * g_laser_num)

        # Simplify the system transfer function
        g_system_sym = sym.simplify(g_system_sym)
        g_system_sym = sym.expand(g_system_sym)
        g_system_sym = sym.simplify(g_system_sym)
        g_system_sym = sym.collect(g_system_sym, s)

        # Print information about g_system_sym
        Routh.printer(g_system_sym, s)

        # Substitute numerical values of kp, kd, and ki into the system transfer function
        g_system_num = g_system_sym.subs(k_p, kp)
        g_system_num = g_system_num.subs(k_d, kd / pid_t_sampling)
        g_system_num = g_system_num.subs(k_i, ki * pid_t_sampling)

        # Print information about g_system_num
        routh_tab = Routh.printer(g_system_num, s)

        first_column = np.array(routh_tab.col(0))  # Get the first column of the Tabulation
        counter_pos = 0  # Counter of positive numbers in tabulation
        counter_neg = 0  # Counter of negative numbers in tabulation

        for i in range(len(first_column)):  # Test each number in the first column
            if first_column[i][0] > 0:
                counter_pos += 1  # Increment the counter if the number is positive

        for i in range(len(first_column)):  # Test each number in the first column
            if first_column[i][0] < 0:
                counter_neg += 1  # Increment the counter if the number is negative

        if counter_pos == len(first_column):
            print('All numbers in the first column are positive!')
            print('The system may be BIBO stable!')
        elif counter_neg == len(first_column):
            print('All numbers in the first column are negative!')
            print('The system may be BIBO stable!')
        else:
            print('Not all numbers in the first column are positive!')
            print('The system is not BIBO stable!')


if __name__ == '__main__':
    print('Please run a different source file.')
