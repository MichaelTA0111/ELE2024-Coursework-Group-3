from scipy.integrate import solve_ivp
import numpy as np


class NonlinearSystem:
    def __init__(self, mass=0.425, gravity=9.81, theta=np.deg2rad(42), c_const=6.815, delta=0.65, k_spring=1880,
                 d_length=0.42, b_damper=10.4, ell_0=0.12, ell_1=0.025, alpha=1.2, resistance=53):
        self.__mass = mass
        self.__gravity = gravity
        self.__theta = theta
        self.__c_const = c_const
        self.__delta = delta
        self.__k_spring = k_spring
        self.__d_length = d_length
        self.__b_damper = b_damper
        self.__ell_0 = ell_0
        self.__ell_1 = ell_1
        self.__alpha = alpha
        self.__resistance = resistance

        self.__x_1 = None
        self.__x_2 = None
        self.__i = None

    def move(self, input_voltage, dt=50):
        initial_values = (self.__x_1, self.__x_2, self.__i)
        state_values = solve_ivp(lambda time, z:
                                 self.ball_dynamics(time, z, input_voltage),
                                 [0, dt],
                                 initial_values,
                                 t_eval=np.linspace(0, dt, 51))

        final_state = state_values.y.T[-1]
        self.__x_1 = final_state[0]
        self.__x_2 = final_state[1]
        self.__i = final_state[2]
        return state_values

    def ball_dynamics(self, time, states, input_voltage):
        x_1_dot = states[1]
        x_2_dot = (5.0 / (3.0 * self.__mass)) * \
                  (self.__mass * self.__gravity * np.sin(self.__theta)
                   + self.__c_const *
                   ((states[2]) / (self.__delta - states[0])) ** 2
                   - self.__k_spring * (states[0] - self.__d_length)
                   - self.__b_damper * states[1])
        i_dot = (1.0 /
                 (self.__ell_0
                  + self.__ell_1
                  * np.exp(-self.__alpha * (self.__delta - states[0]))
                  * (input_voltage - states[2] * self.__resistance)))
        return [x_1_dot, x_2_dot, i_dot]


steel_ball = NonlinearSystem()
