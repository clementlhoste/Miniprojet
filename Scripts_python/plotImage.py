import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button, RadioButtons
import serial
import struct
import sys
import signal
import time
from threading import Thread

#Can be converted into a portable package by using the PyInstaller module
# pip install pyinstaller (need to be used with Python3)
# cf. https://pyinstaller.readthedocs.io/en/v3.3.1/usage.html

goodbye = """
          |\      _,,,---,,_
          /,`.-'`'    -.  ;-;;,_
         |,4-  ) )-,_..;\ (  `'-'
 _______'---''(_/--'__`-'\_)______   ______            _______  _
(  ____ \(  ___  )(  ___  )(  __  \ (  ___ \ |\     /|(  ____ \| |
| (    \/| (   ) || (   ) || (  \  )| (   ) )( \   / )| (    \/| |
| |      | |   | || |   | || |   ) || (__/ /  \ (_) / | (__    | |
| | ____ | |   | || |   | || |   | ||  __ (    \   /  |  __)   | |
| | \_  )| |   | || |   | || |   ) || (  \ \    ) (   | (      |_|
| (___) || (___) || (___) || (__/  )| )___) )   | |   | (____/\ _ 
(_______)(_______)(_______)(______/ |______/    \_/   (_______/(_)                                         
"""

goodbye2 = """
                   /\_/\\
                 =( °w° )=
                   )   (  //
                  (__ __)//
 _____                 _ _                _ 
|  __ \               | | |              | |
| |  \/ ___   ___   __| | |__  _   _  ___| |
| | __ / _ \ / _ \ / _` | '_ \| | | |/ _ \ |
| |_\ \ (_) | (_) | (_| | |_) | |_| |  __/_|
 \____/\___/ \___/ \__,_|_.__/ \__, |\___(_)
                                __/ |       
                               |___/        
"""

#number of samples for one line from the camera
n = 640
#maximum value for an uint8
max_value = 255

#handler when closing the window
def handle_close(evt):
    #we stop the serial thread
    reader_thd.stop()
    print(goodbye)
    
#update the plots
def update_plot():
    if(reader_thd.need_to_update_plot()):
        fig.canvas.draw_idle()
        reader_thd.plot_updated()

#function used to update the plot of the cam data
def update_cam_plot(port):

    cam_data = readUint8Serial(port)
    
    if(len(cam_data)>0):
        cam_plot.set_ydata(cam_data)
        
        graph_cam.relim()
        graph_cam.autoscale()

        reader_thd.tell_to_update_plot()


#reads the data in uint8 from the serial
def readUint8Serial(port):

    state = 0

    while(state != 5):

        #reads 1 byte
        c1 = port.read(1)
        #timeout condition
        if(c1 == b''):
            print('Timout...')
            return [];

        if(state == 0):
            if(c1 == b'S'):
                state = 1
            else:
                state = 0
        elif(state == 1):
            if(c1 == b'T'):
                state = 2
            elif(c1 == b'S'):
                state = 1
            else:
                state = 0
        elif(state == 2):
            if(c1 == b'A'):
                state = 3
            elif(c1 == b'S'):
                state = 1
            else:
                state = 0
        elif(state == 3):
            if(c1 == b'R'):
                state = 4
            elif (c1 == b'S'):
                state = 1
            else:
                state = 0
        elif(state == 4):
            if(c1 == b'T'):
                state = 5
            elif (c1 == b'S'):
                state = 1
            else:
                state = 0

    #reads the size
    #converts as short int in little endian the two bytes read
    size = struct.unpack('<h',port.read(2)) 
    #removes the second element which is void
    size = size[0]  

    #reads the data
    rcv_buffer = port.read(size)
    data = []

    #if we receive the good amount of data, we convert them in float32
    if(len(rcv_buffer) == size):
        i = 0
        while(i < size):
            data.append(struct.unpack_from('<B',rcv_buffer, i))
            i = i+1

        print('received !')
        return data
    else:
        print('Timout...')
        return []

#thread used to control the communication part
class serial_thread(Thread):

    #init function called when the thread begins
    def __init__(self, port):
        Thread.__init__(self)
        self.contReceive = False
        self.alive = True
        self.need_to_update = False

        print('Connecting to port {}'.format(port))

        try:
            self.port = serial.Serial(port, timeout=0.5)
        except:
            print('Cannot connect to the e-puck2')
            sys.exit(0)
    #function called after the init
    def run(self):
        
        while(self.alive):

            if(self.contReceive):
                update_cam_plot(self.port)
            else:
                #flush the serial
                self.port.read(self.port.inWaiting())
                time.sleep(0.1)

    #enables the continuous reading
    def setContReceive(self, val):  
        self.contReceive = True

    #disables the continuous reading
    def stop_reading(self, val):
        self.contReceive = False

    #tell the plot need to be updated
    def tell_to_update_plot(self):
        self.need_to_update = True

    #tell the plot has been updated
    def plot_updated(self):
        self.need_to_update = False

    #tell if the plot need to be updated
    def need_to_update_plot(self):
        return self.need_to_update

    #clean exit of the thread if we need to stop it
    def stop(self):
        self.alive = False
        self.join()
        if(self.port.isOpen()):
            while(self.port.inWaiting() > 0):
                self.port.read(self.port.inWaiting())
                time.sleep(0.01)
            self.port.close()

        
#test if the serial port as been given as argument in the terminal
if len(sys.argv) == 1:
    print('Please give the serial port to use as argument')
    sys.exit(0)

#serial reader thread config
#begins the serial thread
reader_thd = serial_thread(sys.argv[1])
reader_thd.start()

#figure config
fig, ax = plt.subplots(num=None, figsize=(10, 8), dpi=80)
fig.canvas.set_window_title('CamReg plot')
plt.subplots_adjust(left=0.1, bottom=0.25)
fig.canvas.mpl_connect('close_event', handle_close) #to detect when the window is closed and if we do a ctrl-c

#cam graph config with initial plot
graph_cam = plt.subplot(111)
graph_cam.set_ylim([0, max_value])
cam_plot, = plt.plot(np.arange(0,n,1), np.linspace(max_value, max_value, n),lw=1, color='red')
plt.ylabel("camera line")

#timer to update the plot from within the state machine of matplotlib
#because matplotlib is not thread safe...
timer = fig.canvas.new_timer(interval=50)
timer.add_callback(update_plot)
timer.start()

#positions of the buttons, sliders and radio buttons
colorAx             = 'lightgoldenrodyellow'
receiveAx           = plt.axes([0.4, 0.025, 0.1, 0.04])
stopAx              = plt.axes([0.5, 0.025, 0.1, 0.04])

#config of the buttons, sliders and radio buttons
receiveButton           = Button(receiveAx, 'Start reading', color=colorAx, hovercolor='0.975')
stop                    = Button(stopAx, 'Stop reading', color=colorAx, hovercolor='0.975')

#callback config of the buttons, sliders and radio buttons
receiveButton.on_clicked(reader_thd.setContReceive)
stop.on_clicked(reader_thd.stop_reading)

#starts the matplotlib main
plt.show()