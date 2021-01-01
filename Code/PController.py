class PController:
    """
    Class to define the PController object
    """

    def __init__(self,
                 kp=0,
                 ts=0.01):
        """
        Constructor for the PController class
        :param kp: The continuous-time gain for the proportional controller
        :param ts: The sampling time of the controller
        """
        self._kp = kp
        self._ts = ts
        self._error = 0

    def control(self, x_1_bar, set_point=0.):
        """
        Method to calculate the control error
        :param x_1_bar: The measured value of x_1_bar
        :param set_point: The set point value of x_1_bar
        :return: The P control variable
        """
        # Calculate the error
        self._error = set_point - x_1_bar

        # Define u from the proportional controller
        u = self._kp * self._error

        return u
