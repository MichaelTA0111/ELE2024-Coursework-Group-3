import control as ctrl
import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

class LinearSystem:
    def __init__(self, mass=0.425, gravity=9.81, phi=np.deg2rad(42), c_const=6.815, delta=0.65, k_spring=1880,
                 d_length=0.42, b_damper=10.4, ell_0=0.12, ell_1=0.025, alpha=1.2, resistance=53):
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

        self.__x_1_bar = 0.1
        self.__x_2_bar = 0.
        self.__i_bar = 0.

    def move(self, v_bar=0, dt=1, num_points=1001):
        initial_values = (self.__x_1_bar, self.__x_2_bar, self.__i_bar)
        state_values = solve_ivp(lambda time, z:
                                 self.ball_dynamics(time, z, v_bar),
                                 [0, dt],
                                 initial_values,
                                 t_eval=np.linspace(0, dt, num_points))

        final_state = state_values.y.T[-1]
        self.__x_1_bar = final_state[0]
        self.__x_2_bar = final_state[1]
        self.__i_bar = final_state[2]
        return state_values

    def ball_dynamics(self, time, states, v_bar):
        x_1_bar_dot = states[1]
        x_2_bar_dot = self.__d * states[2] + self.__f * states[0] - self.__h * states[1]
        i_bar_dot = self.__n * v_bar - self.__p * states[2]
        return [x_1_bar_dot, x_2_bar_dot, i_bar_dot]

    def get_d(self):
        return self.__d

    def get_f(self):
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


class PidController:

    def __init__(self, kp, kd, ki, ts):
        """
        :param kp: The proportional gain
        :param kd: The derivative gain
        :param ki: The integral gain
        :param ts: The sampling time
        """
        self.__kp = kp
        self.__kd = kd/ts
        self.__ki = ki*ts
        self.__ts = ts
        self.__previous_error = None                    # None i.e. 'Not defined yet'
        self.__sum_errors = 0.0

    def control(self, x, x_set_point=2):
        """
        :param y: The y-position of the car
        :param y_set_point: The desired y-position of the car
        :return:
        """
        error = x_set_point - x                        # Calculates the control error
        ball_voltage = self.__kp*error                # P control

        if self.__previous_error is not None:
            ball_voltage += self.__kd*(error - self.__previous_error)  # D control

        ball_voltage += self.__ki*self.__sum_errors   # I Control

        self.__sum_errors += error
        self.__previous_error = error                   # Means that next time we need the previous error
        return ball_voltage

if __name__ == '__main__':
    dt = 1
    num_points = 1001
    ball = LinearSystem()
    ball_pid = PidController(kp=0.5, kd=0.5, ki=0.1, ts=dt)
    x_cache = np.array([ball.get_x_1_bar()])                        # Inserted current first value of y into the cache
    for k in range(num_points):
        ball_voltage = ball_pid.control(ball.get_x_1_bar())
        ball.move(ball_voltage, dt)
        x_cache = np.vstack((x_cache, [ball.get_x_1_bar()]))

    t_span = dt * np.arange(num_points + 1)
    plt.plot(t_span, x_cache)
    plt.grid()
    plt.show()
