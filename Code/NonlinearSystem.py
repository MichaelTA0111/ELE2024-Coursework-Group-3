from Code.DynamicalSystem import DynamicalSystem
from scipy.integrate import solve_ivp
import numpy as np


class NonlinearSystem(DynamicalSystem):
    """
    Class to define the non-linear system
    """

    def __init__(self, states=None):
        """
        Constructor for the linear system class
        :param states: dictionary containing the following,
            x_1: initial position of ball in metres
            x_2: initial velocity of the ball in metres per second^2
            i: initial value of current in Amps
        """
        super().__init__()  # Construct a dynamical system to inherit from

        # Set the initial conditions of the system
        if states is not None:
            self.__x_1 = states['x_1']
            self.__x_2 = states['x_2']
            self.__i = states['i']
        else:
            self.__x_1 = self._x_1_e
            self.__x_2 = self._x_2_e
            self.__i = self._i_e

    def move(self, voltage=0, dt=1, num_points=1001):
        """
        Method to make the ball object move according to the dynamics of the system
        :param voltage: Input voltage of the system in volts
        :param dt: The difference between the end and start times in seconds
        :param num_points: The resolution of the graph
        :return: The solution describing the system dynamics over time
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

    def ball_dynamics(self, time, states, voltage):
        """
        Method to calculate the values of x_1_dot, x_2_dot, and i_dot
        :param time: Time for the simulation of the system in seconds
        :param states: The current value of x_1, x_2, and i
        :param voltage: Input voltage of the system in volts
        :return: Value of x_1_dot, x_2_dot and i_dot
        """
        x_1_dot = states[1]

        x_2_dot = (5.0 / (3.0 * self._mass)) * \
                  (self._mass * self._gravity * np.sin(self._phi)
                   + self._c_const * ((states[2]) / (self._delta - states[0])) ** 2
                   - self._k_spring * (states[0] - self._d_length)
                   - self._b_damper * states[1])

        i_dot = (1.0 / (self._ell_0 + self._ell_1 * np.exp(-1.0 * self._alpha * (self._delta - states[0])))) * \
                (voltage - (states[2] * self._resistance))

        return [x_1_dot, x_2_dot, i_dot]

    @staticmethod
    def plotter(x_axis, y_axis, file_path, title=None):
        """
        Method to plot a graph of x_1 (m) against time (s)
        :param x_axis: Values of time to be plotted on the x-axis
        :param y_axis: Values of x_1 to be plotted on the y-axis
        :param title: The title of the graph
        :param file_path: The file path where the image will be saved
        :return: None
        """
        super(NonlinearSystem, NonlinearSystem).system_plotter(x_axis,
                                                               y_axis,
                                                               title=title,
                                                               file_path=file_path,
                                                               x_label='Time (s)',
                                                               y_label='${x}_1$ (m)')


if __name__ == '__main__':
    print('Please run a different source file.')
