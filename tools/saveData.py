from pymavlink import mavutil
import signal
from scipy.spatial.transform import Rotation as R

imuConnection = mavutil.mavlink_connection('COM12', baud=115200)

def handler(signum, frame):
    imuConnection.close()
    exit()
signal.signal(signal.SIGINT, handler)

while True:
    msg = imuConnection.recv_match(type='ATTITUDE_QUATERNION')
    if msg is None:
        continue
    att = R.from_quat([msg.q2, msg.q3, msg.q4, msg.q1])
    print(att.as_euler(seq='ZYX', degrees=True))
