from Code.DynamicalSystem import DynamicalSystem, constants
import numpy as np


if __name__ == '__main__':
    # Define the x_1_e variables
    x_1_e_min = constants['d_length'] + \
                (constants['mass'] * constants['gravity'] * np.sin(constants['phi']) / constants['k_spring'])
    x_1_e_max = constants['delta']
    x_1_e_step = (x_1_e_max - x_1_e_min) / 1000

    # Declare the empty arrays for x_1_e and V_e
    x_1_e_array = []
    v_e_array = []

    # Calculate 1001 values of v_e from 1001 evenly distributed values of x_1_e
    for i in range(0, 1001):
        x_1_e = x_1_e_min + i * x_1_e_step  # Define x_1_e
        system = DynamicalSystem(x_1_e=x_1_e)  # Create a dynamical system with equilibrium position x_1_e
        x_1_e_array.append(system.get_x_1_e())  # Append to the x_1_e array
        v_e_array.append(system.get_v_e())  # Append to the v_e array

    v_e_max = max(v_e_array)  # Determine the maximum value of V_e
    index = v_e_array.index(max(v_e_array))  # Determine the index for the maximum value of V_e
    x_e_star = x_1_e_array[index]  # Determine x_e_star from the index of V_e_max

    print('Maximum value of V_e is ' + str(v_e_max) + ' V.')
    print('x_e_star is at ' + str(x_e_star) + ' m.')

    # Plot the graph of V_e against the corresponding values of x_1_e
    DynamicalSystem().system_plotter(x_1_e_array,
                                     v_e_array,
                                     x_label='$x_1^e$ (m)',
                                     y_label='$V^e$ (V)',
                                     file_path='.\\Figures\\ve_against_x1e.svg')
