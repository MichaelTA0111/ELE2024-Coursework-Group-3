from Code.DynamicalSystem import DynamicalSystem
from scipy.integrate import solve_ivp
import numpy as np
import matplotlib.pyplot as plt
from control import TransferFunction as Tf


class LinearSystem(DynamicalSystem):
    """
    Class to define the LinearSystem
    """

    def __init__(self, x_1_bar=0., x_2_bar=0., i_bar=0., v_bar=0.):
        """
        Constructor for the linear system class
        Parameters
        ----------
        x_1_bar: x position - equilibrium x position
        x_2_bar: x position - equilibrium x position
        i_bar: Current - equilibrium current
        v_bar: Input voltage - equilibrium voltage
        """
        super().__init__()

        constant = 5 / (3 * self._mass)
        self.__d = constant * (2 * self._c_const * self._i_e / (self._delta - self._x_1_e) ** 2)
        self.__f = constant * (2 * self._c_const * self._i_e ** 2 / (self._delta - self._x_1_e) ** 3 - self._k_spring)
        self.__h = constant * self._b_damper
        self.__n = 1 / (self._ell_0 + self._ell_1 * np.exp(-self._alpha * (self._delta - self._x_1_e)))
        self.__p = self._resistance * self.__n

        self.__x_1_bar = x_1_bar
        self.__x_2_bar = x_2_bar
        self.__i_bar = i_bar
        self.__v_bar = v_bar

    def move(self, voltage=0, dt=1, num_points=1001):
        """
        Function to move the ball
        Parameters
        ----------
        voltage: Input voltage of the system in volts
        dt: Time for the simulation of the system in seconds
        num_points: Resolution

        Returns: x positions and current
        -------

        """
        initial_values = (self.__x_1_bar, self.__x_2_bar, self.__i_bar)
        if num_points != 0:
            state_values = solve_ivp(lambda time, z:
                                     self.ball_dynamics(time, z, voltage),
                                     [0, dt],
                                     initial_values,
                                     t_eval=np.linspace(0, dt, num_points))
        else:
            state_values = solve_ivp(lambda time, z:
                                     self.ball_dynamics(time, z, voltage),
                                     [0, dt],
                                     initial_values)

        final_state = state_values.y.T[-1]
        self.__x_1_bar = final_state[0]
        self.__x_2_bar = final_state[1]
        self.__i_bar = final_state[2]
        return state_values

    def ball_dynamics(self, time, states, voltage):
        """
        Function to assign the value of x_1_bar_dot, x_2_bar_dot and i_bar_dot
        Parameters
        ----------
        time: Time for the simulation of the system in seconds
        states: The value of the states i.e. x_1_bar_dot, x_2_bar_dot or i_bar_dot
        voltage: Input voltage of the system in volts

        Returns: Value of x_1_bar_dot, x_2_bar_dot and i_bar_dot
        -------

        """
        x_1_bar_dot = states[1]
        x_2_bar_dot = self.__d * states[2] + self.__f * states[0] - self.__h * states[1]
        i_bar_dot = self.__n * (voltage + self.__v_bar) - self.__p * states[2]
        return [x_1_bar_dot, x_2_bar_dot, i_bar_dot]

    def transfer_function(self):
        """
        Function to calculate the value of the transfer function of the liinear system
        :return: The value of the transfer function
        """
        return Tf([self.__d * self.__n],
                                     [1,
                                      (self.__h + self.__p),
                                      (self.__h * self.__p - self.__f),
                                      -(self.__f * self.__p)])

    def get_d(self):
        """
        Getter for the value of the constant d
        :return: The constant d
        """
        return self.__d

    def get_f(self):
        """
        Getter for the value of the constant f
        :return: The constant f
        """
        return self.__f

    def get_h(self):
        """
        Getter for the value of the constant h
        :return: The constant h
        """
        return self.__h

    def get_n(self):
        """
        Getter for the value of the constant n
        :return: The constant n
        """
        return self.__n

    def get_p(self):
        """
        Getter for the value of the constant p
        :return: The constant p
        """
        return self.__p

    def get_x_1_bar(self):
        """
        Getter for the value of the constant x_1_bar
        :return: The constant x_1_bar
        """
        return self.__x_1_bar

    @staticmethod
    def plotter(x_axis, y_axis, file_path=None, title=None):
        """
        Function that plots a graph
        Parameters
        ----------
        x_axis: For values that are required to be plotted on the x-axis
        y_axis: For values that are required to be plotted on the y-axis
        file_path: The file path or location to save the image

        Returns: Nothing
        -------

        """
        plt.plot(x_axis, y_axis)  # Plots the x-axis and y-axis values on a graph
        plt.title(title)
        plt.xlabel('Time (s)')
        plt.ylabel('$\overline{x}_1$ (m)')
        plt.grid()  # Produces a grid on the graph
        plt.savefig(file_path)  # Save the graph
        plt.show()  # Displays the graph


if __name__ == '__main__':
    print('Please run a different source file.')
