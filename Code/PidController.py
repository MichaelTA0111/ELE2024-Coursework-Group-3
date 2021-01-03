from Code.PdController import PdController
from control import TransferFunction as Tf


class PidController(PdController):
    """
    Class to define the PidController object
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
        super().__init__(kp, kd, ts)  # Construct a PdController to inherit from
        self.__ki = ki * ts  # Discrete-time ki
        self.__sum_errors = 0.  # The sum of all previous errors calculated
        self.__u = 0.

    def control(self, x_1_bar, set_point=0.):
        """
        Method to calculate the control error
        :param x_1_bar: The measured value of x_1_bar
        :param set_point: The set point value of x_1_bar
        :return: The PID control variable
        """
        # Use the control function from the PdController
        u = super().control(x_1_bar, set_point)

        # Add to u based on the integral controller
        u += self.__ki * self.__sum_errors

        self.__sum_errors += self._error  # Add the error to the sum of all previous errors
        self.__u = u

        return u

    def transfer_function(self):
        return Tf([self._kd, self._kp, self.__ki], [1, 0])

