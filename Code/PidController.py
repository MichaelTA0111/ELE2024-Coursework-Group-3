from control import TransferFunction as Tf


class PidController:
    """
    Class to define static methods which are used in multiple questions
    """

    @staticmethod
    def pid(kp, ki, kd):
        """
        This function constructs the transfer function of a PID controller with given parameters
        :param kp: The continuous-time gain for the proportional controller
        :param ki: The continuous-time gain for the integral controller
        :param kd: The continuous-time gain for the differential controller
        :return: The transfer function for the PID controller
        """
        diff = Tf([1, 0], 1)
        intgr = Tf(1, [1, 0])
        pid_tf = kp + kd * diff + ki * intgr
        return pid_tf
