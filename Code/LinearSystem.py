from scipy.integrate import solve_ivp
import numpy as np
import matplotlib.pyplot as plt
import control as ctrl


class LinearSystem:
    def __init__(self, mass=0.425, gravity=9.81, phi=np.deg2rad(42), c_const=6.815, delta=0.65, k_spring=1880,
                 d_length=0.42, b_damper=10.4, ell_0=0.12, ell_1=0.025, alpha=1.2, resistance=53, x_1_bar=0.,
                 x_2_bar=0., i_bar=0., v_bar=0.):
        self.__x_1_e = 0.75 * (d_length + (mass * gravity * np.sin(phi) / k_spring)) + 0.25 * delta
        self.__x_2_e = 0.
        self.__i_e = np.sqrt((mass * gravity * np.sin(phi)
                              - k_spring * (self.__x_1_e - d_length))
                             / (-c_const)
                             * (delta - self.__x_1_e) ** 2)
        self.__v_e = self.__i_e * resistance

        self.__d = (5 / (3 * mass)) * (2 * c_const * self.__i_e / (delta - self.__x_1_e) ** 2)
        self.__f = (5 / (3 * mass)) * (2 * c_const * self.__i_e ** 2 / (delta - self.__x_1_e) ** 3 - k_spring)
        self.__h = (5 / (3 * mass)) * b_damper
        self.__n = 1 / (ell_0 + ell_1 * np.exp(-alpha * (delta - self.__x_1_e)))
        self.__p = resistance * self.__n

        self.__x_1_bar = x_1_bar
        self.__x_2_bar = x_2_bar
        self.__i_bar = i_bar
        self.__v_bar = v_bar

    def move(self, voltage=0, dt=1, num_points=1001):
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
        x_1_bar_dot = states[1]
        x_2_bar_dot = self.__d * states[2] + self.__f * states[0] - self.__h * states[1]
        i_bar_dot = self.__n * (voltage + self.__v_bar) - self.__p * states[2]
        return [x_1_bar_dot, x_2_bar_dot, i_bar_dot]

    def transfer_function(self):
        return ctrl.TransferFunction([self.__d * self.__n],
                                     [1, (self.__h + self.__p), (self.__h * self.__p - self.__f),
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
        return self.__h

    def get_n(self):
        return self.__n

    def get_p(self):
        return self.__p

    def get_x_1_bar(self):
        return self.__x_1_bar

    @staticmethod
    def plotter(x_axis, y_axis, file_path):
        plt.plot(x_axis, y_axis)
        plt.xlabel('Time (s)')
        plt.ylabel('$\overline{x}_1$ (m)')
        plt.grid()
        plt.savefig(file_path)  # Save the graph
        plt.show()


if __name__ == '__main__':
    ball = LinearSystem(x_1_bar=0.1)
    ball_trajectory = ball.move()

    ball.plotter(ball_trajectory.t, ball_trajectory.y[0].T, '.\\Figures\\linear_system.svg')
