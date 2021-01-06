from Code.DynamicalSystem import DynamicalSystem
from scipy.integrate import solve_ivp
import numpy as np
import matplotlib.pyplot as plt


class NonlinearSystem(DynamicalSystem):
    """
    Class to define the NonlinearSystem
    """

    def __init__(self, states=None):
        """
        Constructor for the NonlinearSystem class
        Parameters
        ----------
        states: dictionary containing the following,
            x_1: initial position of ball in metres
            x_2: initial velocity of the ball in metres per second^2
            i: initial value of current in Amps
        """
        super().__init__()

        if states is not None:
            self.__x_1 = states['x_1']
            self.__x_2 = states['x_2']
            self.__i = states['i']
        else:
            self.__x_1 = self._x_1_e
            self.__x_2 = self._x_2_e
            self.__i = self._i_e

    def ball_dynamics(self, time, states, input_voltage):
        """
        Function to assign the value of x_1_bar_dot, x_2_bar_dot and i_bar_dot
        Parameters
        ----------
        param time: Time for the simulation of the system in seconds
        states: The value of the states i.e. x_1_bar_dot, x_2_bar_dot or i_bar_dot
        voltage: Input voltage of the system in volts

        :return: Value of x_1_bar_dot, x_2_bar_dot and i_bar_dot
        -------

        """
        x_1_dot = states[1]

        x_2_dot = (5.0 / (3.0 * self._mass)) * \
                  (self._mass * self._gravity * np.sin(self._phi)
                   + self._c_const * ((states[2]) / (self._delta - states[0])) ** 2
                   - self._k_spring * (states[0] - self._d_length)
                   - self._b_damper * states[1])

        i_dot = (1.0 / (self._ell_0 + self._ell_1 * np.exp(-1.0 * self._alpha * (self._delta - states[0])))) * \
                (input_voltage - (states[2] * self._resistance))

        return [x_1_dot, x_2_dot, i_dot]

    def move(self, voltage, dt=1, num_points=1001):
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
        initial_values = (self.__x_1, self.__x_2, self.__i)
        state_values = solve_ivp(lambda time, z:
                                 self.ball_dynamics(time, z, voltage),
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
        plt.ylabel('${x}_1$ (m)')
        plt.grid()  # Produces a grid on the graph
        plt.savefig(file_path)  # Save the graph
        plt.show()  # Displays the graph


if __name__ == '__main__':
    print('Please run a different source file.')
