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
x = [1, 2]
y = [1, 2]

def plot3d():
    fig = plt.figure()
    ax = fig.add_subplot()

    line1, = ax.plot(x, y, label='parametric curve')
    ax.legend()

    while True:
        with lock:
            global do_update
            if do_update:
                print(x)
                do_update = False
                line1.set_xdata(x)
                line1.set_ydata(y)
                plt.xlim([x[-1] - 10, x[-1]])
                plt.ylim([-10,10])
                fig.canvas.draw()
        fig.canvas.flush_events()
        time.sleep(0.1)

t = threading.Thread(target=plot3d)
t.daemon = True
t.start()

start_time = time.time()

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
                    print(acc)     
                    with lock:
                        global do_update
                        do_update = True
                        x.append(time.time() - start_time)
                        y.append(acc[1])
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