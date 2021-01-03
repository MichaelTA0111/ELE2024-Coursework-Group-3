from scipy.integrate import solve_ivp
import numpy as np
import matplotlib.pyplot as plt


class NonlinearSystem:
    """
        Class to define the NonlinearSystem
    """
    def __init__(self, mass=0.425, gravity=9.81, phi=np.deg2rad(42), c_const=6.815, delta=0.65, k_spring=1880,
                 d_length=0.42, b_damper=10.4, ell_0=0.12, ell_1=0.025, alpha=1.2, resistance=53):
        """
        Constructor for the NonlinearSystem class
        Parameters
        ----------
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
        self.__mass = mass
        self.__gravity = gravity
        self.__phi = phi
        self.__c_const = c_const
        self.__delta = delta
        self.__k_spring = k_spring
        self.__d_length = d_length
        self.__b_damper = b_damper
        self.__ell_0 = ell_0
        self.__ell_1 = ell_1
        self.__alpha = alpha
        self.__resistance = resistance

        self.__x_1 = 0.75 * (d_length + (mass * gravity * np.sin(phi) / k_spring)) + 0.25 * delta
        self.__x_2 = 0.
        self.__i = 0.

    def move(self, input_voltage=0, dt=1, num_points=1001):
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
                                 initial_values,
                                 t_eval=np.linspace(0, dt, num_points))

        final_state = state_values.y.T[-1]
        self.__x_1 = final_state[0]
        self.__x_2 = final_state[1]
        self.__i = final_state[2]
        return state_values

    def ball_dynamics(self, time, states, input_voltage):
        """
        Function to assign the value of x_1_bar_dot, x_2_bar_dot and i_bar_dot
        Parameters
        ----------
        time: Time for the simulation of the system in seconds
        states: The value of the states i.e. x_1_bar_dot, x_2_bar_dot or i_bar_dot
        input_voltage: Input voltage of the system in volts

        Returns: Value of x_1_bar_dot, x_2_bar_dot and i_bar_dot
        -------

        """
        x_1_dot = states[1]

        x_2_dot = (5.0 / (3.0 * self.__mass)) * \
                  (self.__mass * self.__gravity * np.sin(self.__phi)
                   + self.__c_const * ((states[2]) / (self.__delta - states[0])) ** 2
                   - self.__k_spring * (states[0] - self.__d_length)
                   - self.__b_damper * states[1])

        i_dot = ((1.0 / (self.__ell_0 + self.__ell_1
                         * np.exp(-1 * self.__alpha * (self.__delta - states[0]))))
                 * (input_voltage - (states[2] * self.__resistance)))

        self.__x_1, self.__x_2, self.__i = [x_1_dot, x_2_dot, i_dot]

        return [x_1_dot, x_2_dot, i_dot]

    def get_i(self):
        """
        Getter for the value of the constant i
        :return: The constant i
        """
        return self.__i

    def get_resistance(self):
        """
        Getter for the value of resistance
        :return: Resistance
        """
        return self.__resistance

    @staticmethod
    def plotter(x_axis, y_axis, file_path):
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
        plt.xlabel('Time (s)')
        plt.ylabel('${x}_1$ (m)')
        plt.grid()  # Produces a grid on the graph
        plt.savefig(file_path)  # Save the graph
        plt.show()  # Displays the graph


if __name__ == '__main__':
    ball = NonlinearSystem()
    ball_voltage = ball.get_i() * ball.get_resistance()  # V = IR
    ball_trajectory = ball.move(ball_voltage)

    # Plot graph
    ball.plotter(ball_trajectory.t, ball_trajectory.y[0].T, '.\\Figures\\nonlinear_system.svg')  # x position v.s time
