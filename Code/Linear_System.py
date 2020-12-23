from scipy.integrate import solve_ivp
import numpy as np


class LinearSystem:
    def __init__(self, mass=0.425, gravity=9.81, slope_angle=np.deg2rad(42), c=6815, inductor_distance=0.65,
                 spring_constant=1880, natural_spring_length=0.42, damping_coefficient=10.4, nominal_inductance=0.12,
                 ell_1=0.025, alpha=1.2, resistance=53):
        self.__x_1_e = 0.75 * (natural_spring_length + mass * gravity * np.sin(slope_angle))
        self.__x_2_e = 0
        self.__i_e = np.sqrt((mass * gravity * np.sin(slope_angle)
                             - spring_constant * (self.__x_1_e - natural_spring_length))
                             / (-c)
                             * (inductor_distance - self.__x_1_e)**2)
        self.__v_e = self.__i_e * resistance

        self.__d = (5 / (3 * mass)) * (2 * c * self.__i_e / (inductor_distance - self.__x_1_e)**2)
        self.__f = (5 / (3 * mass)) * (2 * c * self.__i_e**2 / (inductor_distance - self.__x_1_e)**3 - spring_constant)
        self.__h = (5 / (3 * mass)) * damping_coefficient
        self.__n = 1 / (nominal_inductance + ell_1 * np.exp(-alpha * (inductor_distance - self.__x_1_e)))
        self.__p = resistance * self.__n

        self.__x_1_bar = None
        self.__x_2_bar = None
        self.__i_bar = None

    def move(self, v_bar, dt):
        initial_values = (self.__x_1_bar, self.__x_2_bar, self.__i_bar)
        state_values = solve_ivp(lambda time, z:
                                 self.ball_dynamics(time, z, v_bar),
                                 [0, dt],
                                 initial_values,
                                 t_eval=np.linspace(0, dt, 1000))

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


steel_ball = LinearSystem()
