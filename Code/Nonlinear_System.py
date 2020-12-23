from scipy.integrate import solve_ivp
import numpy as np


class NonlinearSystem:
    def __init__(self, mass=0.425, gravity=9.81, slope_angle=np.deg2rad(42), c=6815, inductor_distance=0.65,
                 spring_constant=1880, natural_spring_length=0.42, damping_coefficient=10.4, nominal_inductance=0.12,
                 ell_1=0.025, alpha=1.2, resistance=53):
        self.__mass = mass
        self.__gravity = gravity
        self.__slope_angle = slope_angle
        self.__c = c
        self.__inductor_distance = inductor_distance
        self.__spring_constant = spring_constant
        self.__natural_spring_length = natural_spring_length
        self.__damping_coefficient = damping_coefficient
        self.__nominal_inductance = nominal_inductance
        self.__ell_1 = ell_1
        self.__alpha = alpha
        self.__resistance = resistance

        self.__x_1 = None
        self.__x_2 = None
        self.__i = None

    def move(self, input_voltage, dt):
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
                  (self.__mass * self.__gravity * np.sin(self.__slope_angle)
                   + self.__c *
                   ((states[2]) / (self.__inductor_distance - states[0])) ** 2
                   - self.__spring_constant * (states[0] - self.__natural_spring_length)
                   - self.__damping_coefficient * states[1])
        i_dot = (1.0 /
                 (self.__nominal_inductance
                  + self.__ell_1
                  * np.exp(-self.__alpha * (self.__inductor_distance - states[0]))
                  * (input_voltage - states[2] * self.__resistance)))
        return [x_1_dot, x_2_dot, i_dot]


steel_ball = NonlinearSystem()
