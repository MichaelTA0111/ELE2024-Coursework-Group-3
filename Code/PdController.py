from Code.PController import PController


class PdController(PController):
    """
    Class to define the PdController object
    """

    def __init__(self,
                 kp=0,
                 kd=0,
                 ts=0.01):
        """
        Constructor for the PdController class
        :param kp: The continuous-time gain for the proportional controller
        :param kd: The continuous-time gain for the differential controller
        :param ts: The sampling time of the controller
        """
        super().__init__(kp, ts)  # Construct a PController to inherit from
        self._kd = kd / ts  # Discrete-time kd
        self._error_previous = None  # The error recorded the previous time it was calculated

    def control(self, x_1_bar, set_point=0.):
        """
        Method to calculate the control error
        :param x_1_bar: The measured value of x_1_bar
        :param set_point: The set point value of x_1_bar
        :return: The PD control variable
        """
        # Use the control function from the PController
        u = super().control(x_1_bar, set_point)

        # Add to u based on the differential controller
        if self._error_previous is not None:
            u += self._kd * (self._error - self._error_previous)

        self._error_previous = self._error  # Store the calculated error as the previous error for future use

        return u
