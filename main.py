import matplotlib.pyplot as plt
import numpy as np
import time
import threading
import serial
import sys
import struct
import quaternion

plt.ion()

lock = threading.Lock()
do_update = False
x = []
y = []
z = []

def plot3d():
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')

    line1, = ax.plot(x, y, z, label='parametric curve')
    ax.legend()

    while True:
        with lock:
            global do_update
            if do_update:
                do_update = False
                line1.set_xdata(x)
                line1.set_ydata(y)
                line1.set_3d_properties(z)
                fig.canvas.draw()
        fig.canvas.flush_events()
        time.sleep(0.1)

t = threading.Thread(target=plot3d)
t.daemon = True
t.start()









pos = np.array([0, 0, 0])
last_t = None
acc = np.array([0, 0, 0])
vel = np.array([0, 0, 0])
rot = np.array([0, 0, 0, 0])

def update_pos():
    current_t = time.time()
    global pos, last_t, acc, vel, rot
    if (last_t is None):
        last_t = current_t
    # rot_q = np.quaternion(rot[0], rot[1], rot[2], rot[3])
    # acc_q = np.quaternion(0, acc[0], acc[1], acc[1])
    # roted_acc_q = np.conjugate(rot_q) * acc_q * rot_q
    # roted_acc = np.array([roted_acc_q.x, roted_acc_q.y, roted_acc_q.z])
    # print(acc, roted_acc)
    # print(acc_q, roted_acc_q)
    # print(rot_q, acc, roted_acc)
    roted_acc = acc
    vel = (vel + (current_t - last_t) * roted_acc) * 0.9999
    pos = pos + (current_t - last_t) * vel
    # print(vel)
    # print(acc)
    # print()
    print(current_t - last_t)
    with lock:
        global do_update
        do_update = True
        x.append(pos[0])
        y.append(pos[1])
        z.append(pos[2])
    last_t = current_t

class ImuController:
    COMM_PAYLOAD_LEN = 16
    COMM_TYPE_PING = 0x00
    COMM_TYPE_PONG = 0x01
    COMM_TYPE_ACC = 0x02
    COMM_TYPE_GYRO = 0x03
    COMM_TYPE_ROTA = 0x04

    def __init__(self, name):
        self.ser = serial.Serial(name, 921600, timeout=None)

        self.quit = threading.Event()
        self.t = threading.Thread(target=self.recv_thread)
        self.t.daemon = True
        self.t.start()

    def recv_thread(self):
        try:
            while not self.quit.is_set():
                data = self.ser.read(1)
                if not (data[0] == 0x5A): # 逐步同步
                    sys.stdout.buffer.write(data) # print plain text on screen
                    sys.stdout.flush()
                    continue
                
                data += self.ser.read(1 + self.COMM_PAYLOAD_LEN)
                assert(len(data) == 1 + 1 + self.COMM_PAYLOAD_LEN)

                if data[1] == self.COMM_TYPE_PING:
                    pass
                elif data[1] == self.COMM_TYPE_PONG:
                    pass
                elif data[1] == self.COMM_TYPE_ACC:
                    parsed = np.array(struct.unpack('>xxfffxxxx', data))
                    global acc
                    acc = parsed
                    update_pos()
                elif data[1] == self.COMM_TYPE_GYRO:
                    pass
                elif data[1] == self.COMM_TYPE_ROTA:
                    parsed = np.array(struct.unpack('>xxffff', data))
                    global rot
                    rot = parsed
                    
                    # print(rot)
                    # exit()
                    
        # except Exception:
        #     pass
        finally:
            print("thread exiting")
        
    def __del__(self):
        self.quit.set()
        self.ser.close()

ImuController("COM4")

while True:
    time.sleep(0.1)