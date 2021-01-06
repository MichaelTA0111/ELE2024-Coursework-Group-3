from Code.DynamicalSystem import DynamicalSystem
from scipy.integrate import solve_ivp
import numpy as np
from control import TransferFunction as Tf


class LinearSystem(DynamicalSystem):
    """
    Class to define the linear system
    """

    def __init__(self, x_1_bar=0., x_2_bar=0., i_bar=0., v_bar=0.):
        """
        Constructor for the linear system class
        :param x_1_bar: x position relative to the equilibrium x position
        :param x_2_bar: Speed of the ball relative to the equilibrium speed of the ball
        :param i_bar: Current relative to the equilibrium current
        :param v_bar: Input voltage relative to the equilibrium voltage
        """
        super().__init__()  # Construct a dynamical system to inherit from

        # Calculate constants used in the linear equations
        constant = 5 / (3 * self._mass)
        self.__d = constant * (2 * self._c_const * self._i_e / (self._delta - self._x_1_e) ** 2)
        self.__f = constant * (2 * self._c_const * self._i_e ** 2 / (self._delta - self._x_1_e) ** 3 - self._k_spring)
        self.__h = constant * self._b_damper
        self.__n = 1 / (self._ell_0 + self._ell_1 * np.exp(-self._alpha * (self._delta - self._x_1_e)))
        self.__p = self._resistance * self.__n

        # Set the initial conditions of the system
        self.__x_1_bar = x_1_bar
        self.__x_2_bar = x_2_bar
        self.__i_bar = i_bar
        self.__v_bar = v_bar

    def move(self, voltage=0, dt=1, num_points=1001):
        """
        Method to make the ball object move according to the dynamics of the system
        :param voltage: Input voltage of the system in volts
        :param dt: The difference between the end and start times in seconds
        :param num_points: The resolution of the graph
        :return: The solution describing the system dynamics over time
        """
        initial_values = (self.__x_1_bar, self.__x_2_bar, self.__i_bar)
        state_values = solve_ivp(lambda time, z:
                                 self.ball_dynamics(time, z, voltage),
                                 [0, dt],
                                 initial_values,
                                 t_eval=np.linspace(0, dt, num_points))

        final_state = state_values.y.T[-1]
        self.__x_1_bar = final_state[0]
        self.__x_2_bar = final_state[1]
        self.__i_bar = final_state[2]
        return state_values

    def ball_dynamics(self, time, states, voltage):
        """
        Method to calculate the values of x_1_bar_dot, x_2_bar_dot, and i_bar_dot
        :param time: Time for the simulation of the system in seconds
        :param states: The current value of x_1_bar, x_2_bar, and i_bar
        :param voltage: Input voltage of the system in volts
        :return: Value of x_1_bar_dot, x_2_bar_dot and i_bar_dot
        """
        x_1_bar_dot = states[1]
        x_2_bar_dot = self.__d * states[2] + self.__f * states[0] - self.__h * states[1]
        i_bar_dot = self.__n * (voltage + self.__v_bar) - self.__p * states[2]
        return [x_1_bar_dot, x_2_bar_dot, i_bar_dot]

    def transfer_function(self):
        """
        Method to calculate the value of the transfer function of the liinear system
        :return: The value of the transfer function
        """
        return Tf([self.__d * self.__n],
                  [1,
                   (self.__h + self.__p),
                   (self.__h * self.__p - self.__f),
                   -(self.__f * self.__p)])

    def get_x_1_bar(self):
        """
        Getter for the value of x_1_bar
        :return: The variable x_1_bar
        """
        return self.__x_1_bar

    @staticmethod
    def plotter(x_axis, y_axis, title=None, file_path=None):
        """
        Method to plot a graph of x_1_bar (m) against time (s)
        :param x_axis: Values of time to be plotted on the x-axis
        :param y_axis: Values of x_1_bar to be plotted on the y-axis
        :param title: The title of the graph
        :param file_path: The file path where the image will be saved
        :return: None
        """
        super(LinearSystem, LinearSystem).system_plotter(x_axis,
                                                         y_axis,
                                                         title=title,
                                                         file_path=file_path,
                                                         x_label='Time (s)',
                                                         y_label='$\overline{x}_1$ (m)')


if __name__ == '__main__':
    print('Please run a different source file.')
