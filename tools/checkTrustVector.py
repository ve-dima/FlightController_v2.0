from pymavlink import mavutil
import signal
import numpy as np
from scipy.spatial.transform import Rotation as R


def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)


def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'::

            >>> angle_between((1, 0, 0), (0, 1, 0))
            1.5707963267948966
            >>> angle_between((1, 0, 0), (1, 0, 0))
            0.0
            >>> angle_between((1, 0, 0), (-1, 0, 0))
            3.141592653589793
    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))


np.set_printoptions(precision=2, floatmode='fixed', suppress=True)
imuConnection = mavutil.mavlink_connection('COM12', baud=115_200)


def handler(signum, frame):
    imuConnection.close()
    exit()


signal.signal(signal.SIGINT, handler)

while True:
    msg = imuConnection.recv_match(type='ACTUATOR_CONTROL_TARGET')
    if msg is None:
        continue

    motorVec = np.array([
        msg.controls[0] - msg.controls[1] + msg.controls[2] - msg.controls[3],
        -msg.controls[0] + msg.controls[1] + msg.controls[2] - msg.controls[3],
        -msg.controls[0] - msg.controls[1] + msg.controls[2] + msg.controls[3],
    ])
    motorVec /= 4
    targetVec = np.array([msg.controls[4], msg.controls[5], msg.controls[6]])

    collectiveTrust = sum(msg.controls[:4]) / 4
    minimalTrust = min(msg.controls[:4])

    print(motorVec, targetVec,
          'A err: {:.2f}° '.format(np.rad2deg(
              angle_between(motorVec, targetVec))),
          'C thr: {:.2f} '.format(collectiveTrust),
          'M tht: {:.2f}'.format(minimalTrust))
