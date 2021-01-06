from scipy.integrate import solve_ivp
import numpy as np
import matplotlib.pyplot as plt


class NonlinearSystem:
    """
    Class to define the NonlinearSystem
    """
    def __init__(self, states, attributes):
        """
        Constructor for the NonlinearSystem class
        Parameters
        ----------
        states: dictionary containing the following,
            x_1: initial position of ball in metres
            x_2: initial velocity of the ball in metres per second^2
            i: initial value of current in Amps
        attributes: dictionary containing the following,
            mass: Mass of the steel ball in Kg
            gravity: acceleration due to gravity of the system in metres per second^2
            phi: Angle of the slope in radians
            c_const: Constant in gm/a^2s^2
            delta: Distance between the centre of the electromagnet and the wall in metres
            k_spring: Spring constant
            d_length: Natural length of the spring in Newtons per metre
            b_damper: Viscous damping coefficient in Newtons seconds per metre
            ell_0: Nominal inductance in henrys
            ell_1: Inductor constant in henrys
            alpha: Constant in per metre
            resistance: Resistance of the electromagnet in ohms
        """
        self.__mass = attributes['mass']
        self.__gravity = attributes['gravity']
        self.__phi = attributes['phi']
        self.__c_const = attributes['c_const']
        self.__delta = attributes['delta']
        self.__k_spring = attributes['k_spring']
        self.__d_length = attributes['d_length']
        self.__b_damper = attributes['b_damper']
        self.__ell_0 = attributes['ell_0']
        self.__ell_1 = attributes['ell_1']
        self.__alpha = attributes['alpha']
        self.__resistance = attributes['resistance']

        self.__x_1 = states["x_1"]
        self.__x_2 = states["x_2"]
        self.__i = states["i"]

    def ball_dynamics(self, time, states, input_voltage):
        """
        Function to assign the value of x_1_bar_dot, x_2_bar_dot and i_bar_dot
        Parameters
        ----------
        :param time: Time for the simulation of the system in seconds
        states: The value of the states i.e. x_1_bar_dot, x_2_bar_dot or i_bar_dot
        input_voltage: Input voltage of the system in volts

        :return: Value of x_1_bar_dot, x_2_bar_dot and i_bar_dot
        -------

        """
        x_1_dot = states[1]

        x_2_dot = (5.0 / (3.0 * self.__mass)) * \
                  (self.__mass * self.__gravity * np.sin(self.__phi)
                   + self.__c_const * ((states[2]) / (self.__delta - states[0])) ** 2
                   - self.__k_spring * (states[0] - self.__d_length)
                   - self.__b_damper * states[1])

        i_dot = (1.0 / (self.__ell_0 + self.__ell_1 * np.exp(-1.0 * self.__alpha * (self.__delta - states[0])))) *\
                (input_voltage - (states[2] * self.__resistance))

        return [x_1_dot, x_2_dot, i_dot]

    def move(self, input_voltage, dt=1, num_points=1001):
        """
        Function to move the ball
        Parameters
        ----------
        input_voltage: Input voltage of the system in volts
        dt: Time for the simulation of the system in seconds
        num_points: Resolution

        Returns: x positions and current
        -------

        """
        initial_values = (self.__x_1, self.__x_2, self.__i)
        state_values = solve_ivp(lambda time, z:
                                 self.ball_dynamics(time, z, input_voltage),
                                 [0, dt],
                                 initial_values, method='Radau',
                                 t_eval=np.linspace(0, dt, num_points))

        final_state = state_values.y.T[-1]
        self.__x_1 = final_state[0]
        self.__x_2 = final_state[1]
        self.__i = final_state[2]
        return state_values

    @staticmethod
    def plotter(x_axis, y_axis, file_path, title=None):
        plt.plot(x_axis, y_axis)  # Plots the x-axis and y-axis values on a graph
        plt.title(title)
        plt.xlabel('Time (s)')
        plt.ylabel('${x}_1$ (m)')
        plt.grid()  # Produces a grid on the graph
        plt.savefig(file_path)  # Save the graph
        plt.show()  # Displays the graph


if __name__ == '__main__':
    print('Please run a different source file.')
