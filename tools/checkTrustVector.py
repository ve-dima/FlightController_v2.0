from pymavlink import mavutil
import signal
import numpy as np
from scipy.spatial.transform import Rotation as R

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
    targetVec = np.array([msg.controls[4], msg.controls[5], msg.controls[6]])
