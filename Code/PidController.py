from control import TransferFunction as Tf


class PidController:
    """
    Class to define the PID Controller object
    """

    def __init__(self,
                 kp=0,
                 kd=0,
                 ki=0,
                 ts=0.01):
        """
        Constructor for the PidController class
        :param kp: The continuous-time gain for the proportional controller
        :param kd: The continuous-time gain for the differential controller
        :param ki: The continuous-time gain for the integral controller
        :param ts: The sampling time of the controller
        """
        self.__kp = kp
        self.__kd = kd / ts  # Discrete-time kd
        self.__ki = ki * ts  # Discrete-time ki

        self.__error = 0.
        self.__error_previous = None  # The error recorded the previous time it was calculated
        self.__sum_errors = 0.  # The sum of all previous errors calculated

        self.__ts = ts

    def control(self, x_1_bar, set_point=0.):
        """
        Method to calculate the control variable
        :param x_1_bar: The measured value of x_1_bar
        :param set_point: The set point value of x_1_bar
        :return: The PID control variable
        """
        # Calculate the error
        self.__error = set_point - x_1_bar

        # Define the control variable from the proportional controller
        control = self.__kp * self.__error

        # Add to the control variable based on the differential controller
        if self.__error_previous is not None:
            control += self.__kd * (self.__error - self.__error_previous)

        # Store the calculated error as the previous error for future use
        self.__error_previous = self.__error

        # Add to the control variable based on the integral controller
        control += self.__ki * self.__sum_errors

        # Add the error to the sum of all previous errors
        self.__sum_errors += self.__error

        return control

    def transfer_function(self):
        """
        Function to calculate the value of the transfer function of the PID controller
        :return: The value of the transfer function
        """
        return Tf([self.__kd, self.__kp, self.__ki], [1, 0])


if __name__ == '__main__':
    print('Please run a different source file.')
