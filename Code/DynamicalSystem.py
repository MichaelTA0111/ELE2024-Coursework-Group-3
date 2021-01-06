import numpy as np
import matplotlib.pyplot as plt

# Dictionary containing values for the dynamical system
constants = {
    'mass': 0.425,
    'gravity': 9.81,
    'phi': np.deg2rad(42.0),
    'c_const': 6.815,
    'delta': 0.65,
    'k_spring': 1880,
    'd_length': 0.42,
    'b_damper': 10.4,
    'ell_0': 0.12,
    'ell_1': 0.025,
    'alpha': 1.2,
    'resistance': 53
}


class DynamicalSystem:
    """
    Class to define the dynamical system
    """

    def __init__(self, attributes=None):
        """
        Constructor for the dynamical system parent class
        :param attributes: dictionary containing the following;
            mass: Mass of the steel ball in Kg
            gravity: Acceleration due to gravity of the system in metres per second^2
            phi: Angle of the slope in radians
            c_const: Constant in kg m / (a^2 s^2)
            delta: Distance between the centre of the electromagnet and the wall in metres
            k_spring: Spring constant
            d_length: Natural length of the spring in Newtons per metre
            b_damper: Viscous damping coefficient in Newtons seconds per metre
            ell_0: Nominal inductance in henrys
            ell_1: Inductor constant in henrys
            alpha: Constant in per metre
            resistance: Resistance of the electromagnet in ohms
        """
        if attributes is None:
            attributes = constants

        self._mass = attributes['mass']
        self._gravity = attributes['gravity']
        self._phi = attributes['phi']
        self._c_const = attributes['c_const']
        self._delta = attributes['delta']
        self._k_spring = attributes['k_spring']
        self._d_length = attributes['d_length']
        self._b_damper = attributes['b_damper']
        self._ell_0 = attributes['ell_0']
        self._ell_1 = attributes['ell_1']
        self._alpha = attributes['alpha']
        self._resistance = attributes['resistance']

        self._x_1_e = 0.75 \
                      * (self._d_length + (self._mass * self._gravity * np.sin(self._phi) / self._k_spring)) \
                      + 0.25 * self._delta
        self._x_2_e = 0.
        self._i_e = np.sqrt((self._mass * self._gravity * np.sin(self._phi)
                             - self._k_spring * (self._x_1_e - self._d_length))
                            / (-self._c_const)
                            * (self._delta - self._x_1_e) ** 2)
        self._v_e = self._i_e * self._resistance

    def get_x_1_e(self):
        """
        Getter for the value of the constant x_1_e
        :return: The constant x_1_e
        """
        return self._x_1_e

    def get_x_2_e(self):
        """
        Getter for the value of the constant x_2_e
        :return: The constant x_2_e
        """
        return self._x_2_e

    def get_i_e(self):
        """
        Getter for the value of the constant i_e
        :return: The constant i_e
        """
        return self._i_e

    def get_v_e(self):
        """
        Getter for the value of the constant v_e
        :return: The constant v_e
        """
        return self._v_e

    @staticmethod
    def system_plotter(x_axis, y_axis, title=None, x_label=None, y_label=None, file_path=None):
        """
        Method to plot a graph of x_1 (m) against time (s)
        :param x_axis: Values of time to be plotted on the x-axis
        :param y_axis: Values of x_1 to be plotted on the y-axis
        :param title: The title of the graph
        :param x_label: The label of the x-axis
        :param y_label: The label of the y-axis
        :param file_path: The file path where the image will be saved
        :return: None
        """
        plt.title(title)
        plt.plot(x_axis, y_axis)  # Plots the x-axis and y-axis values on a graph
        plt.xlabel(x_label)
        plt.ylabel(y_label)
        plt.grid()  # Produces a grid on the graph
        plt.savefig(file_path)  # Save the graph
        plt.show()  # Displays the graph


if __name__ == '__main__':
    print('Please run a different source file.')
