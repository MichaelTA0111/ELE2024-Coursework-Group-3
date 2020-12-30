from scipy.integrate import solve_ivp
import numpy as np
import matplotlib.pyplot as plt


class NonlinearSystem:
    def __init__(self, mass=0.425, gravity=9.81, phi=np.deg2rad(42), c_const=6.815, delta=0.65, k_spring=1880,
                 d_length=0.42, b_damper=10.4, ell_0=0.12, ell_1=0.025, alpha=1.2, resistance=53):
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
        self.resistance = resistance

        self.__x_1 = 0.75 * (d_length + (mass * gravity * np.sin(phi) / k_spring)) + 0.25 * delta
        self.__x_2 = 0
        # self.i = np.sqrt((mass * gravity * np.sin(phi) - k_spring * (self.__x_1 - d_length))
        #                  / (-c_const)
        #                  * (delta - self.__x_1) ** 2)
        self.i = 0
        # self.__v = self.__i * resistance

    def move(self, input_voltage=0, dt=10, num_points=1001):
        initial_values = (self.__x_1, self.__x_2, self.i)
        state_values = solve_ivp(lambda time, z:
                                 self.ball_dynamics(time, z, input_voltage),
                                 [0, dt],
                                 initial_values,
                                 t_eval=np.linspace(0, dt, num_points))

        final_state = state_values.y.T[-1]
        self.__x_1 = final_state[0]
        self.__x_2 = final_state[1]
        self.i = final_state[2]
        return state_values

    def ball_dynamics(self, time, states, input_voltage):
        x_1_dot = states[1]

        x_2_dot = (5.0 / (3.0 * self.__mass)) * \
                  (self.__mass * self.__gravity * np.sin(self.__phi)
                   + self.__c_const * ((states[2]) / (self.__delta - states[0])) ** 2
                   - self.__k_spring * (states[0] - self.__d_length)
                   - self.__b_damper * states[1])

        i_dot = ((1.0 / (self.__ell_0 + self.__ell_1
                         * np.exp(-1 * self.__alpha * (self.__delta - states[0]))))
                 * (input_voltage - (states[2] * self.resistance)))

        self.__x_1, self.__x_2, self.i = [x_1_dot, x_2_dot, i_dot]

        return [x_1_dot, x_2_dot, i_dot]

    @staticmethod
    def plotter(x_axis, y_axis, file_path):
        plt.plot(x_axis, y_axis)
        plt.xlabel('Time (s)')
        plt.ylabel('${x}_1$ (m)')
        plt.grid()
        plt.title("Nonlinear System")
        plt.savefig(file_path)  # Save the graph
        plt.show()


if __name__ == '__main__':
    ball = NonlinearSystem()
    input_voltage = ball.i * ball.resistance  # equil voltage
    ball_trajectory = ball.move(input_voltage)

    print(ball_trajectory.y.T[-1])

    ball.plotter(ball_trajectory.t, ball_trajectory.y[0].T, '.\\Figures\\nonlinear_system.svg')
