"""
6-DOF Robotic Arm - legacy PyQt5 desktop controller.

Standalone task-space/joint-space control GUI used during the serial-link
control phase of the project (superseded by the Qt/QML GUI in ../content).
Uses ikpy for numerical inverse kinematics and pyserial to talk to the
Arduino Mega 2560 over UART.

Author: Luca Obwegs
"""

import sys
from tkinter.messagebox import showinfo
import ikpy.chain
from threading import *
import time
import numpy as np
import serial
from scipy.spatial.transform import Rotation as R
import numpy as np
from PyQt5.QtWidgets import (
    QApplication, QMainWindow
)
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from robot_gui import Ui_MainWindow
from PyQt5 import QtWidgets
#from urdfpy import URDF

class Window(QMainWindow, Ui_MainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setupUi(self)
        
             
        #motion repeat list
        self.mrLst = [] #contains the position that should be repeated

        #plot
        self.fig = Figure()
        self.canvas = FigureCanvas(self.fig)
        self.ax = self.fig.add_subplot(111, projection='3d')
        layout = QtWidgets.QVBoxLayout(self.frame_rviz)
        layout.addWidget(self.canvas)

        #variables
        self.slowFactor = 2
        self.stopRecord = False
        self.maxRecordTime = 100
        self.time_step = 0.01
        self.velocity = 150 #deg/s
        self.acceleration = 50 #deg/s^2
        self.vmax = 0
        self.baudRate = 1000000
        self.x_min = -360
        self.y_min = -360
        self.z_min = -100
        self.x_max = 360
        self.y_max = 360
        self.z_max = 498
        self.j1_min = -360
        self.j1_max = 360
        self.j2_min = -360
        self.j2_max = 360
        self.j3_min = -360
        self.j3_max = 360
        self.j4_min = -360
        self.j4_max = 360
        self.j5_min = -360
        self.j5_max = 360
        self.j6_min = -360
        self.j6_max = 360
        self.j1_zero_pos = 0
        self.j1_pos = self.j1_zero_pos
        self.j2_zero_pos = 90
        self.j2_pos = self.j2_zero_pos
        self.j3_zero_pos = -180
        self.j3_pos = self.j3_zero_pos
        self.j4_zero_pos = 0
        self.j4_pos = self.j4_zero_pos
        self.j5_zero_pos = 0
        self.j5_pos = self.j5_zero_pos
        self.j6_zero_pos = 0
        self.j6_pos = self.j6_zero_pos
        self.x_zero_pos = 0
        self.x_pos = self.x_zero_pos
        self.y_zero_pos = 5.02835
        self.y_pos = self.y_zero_pos
        self.z_zero_pos = 224
        self.z_pos = self.z_zero_pos
        self.phi_zero_pos = 0
        self.phi_pos = self.phi_zero_pos
        self.theta_zero_pos = 0
        self.theta_pos = self.theta_zero_pos
        self.psi_zero_pos = 0
        self.psi_pos = self.psi_zero_pos
        self.step_size = 10
        self.orient_mode = "all"
        self.mm2m = 1e-3
        self.deg2rad = np.pi/180
        self.zero_pos = [0,0,1.57,0, -3.14,0,0,0]
        self.start_position = [[1,0,0,0],[0,0,1,0.00502835], [0,1,0,0.20960934], [0,0,0,1]]
        self.init_run = True
        self.ik_iterations = 5
        self.comPort = 'COM7'
        self.rotTreshHold = 0.1*self.deg2rad; #in rad
        self.ang_1 = 0
        self.ang_2 = 0
        self.ang_3 = 0
        self.ang_4 = 0
        self.ang_5 = 0
        self.ang_6 = 0
        
        #connect uart
        self.ser = self.connectUart()

        #import robot urdf
        self.my_chain = ikpy.chain.Chain.from_urdf_file("arm_urdf.urdf",active_links_mask=[False, True, True, False, True, True, True, True])
        print("URDF imported successfully")
        
        #set sliders
        self.slider_x.setRange(self.x_min, self.x_max)
        self.slider_y.setRange(self.y_min, self.y_max)
        self.slider_z.setRange(self.z_min, self.z_max)
        self.slider_j1.setRange(self.j1_min, self.j1_max)
        self.slider_j2.setRange(self.j2_min, self.j2_max)
        self.slider_j3.setRange(self.j3_min, self.j3_max)
        self.slider_j4.setRange(self.j4_min, self.j4_max)
        self.slider_j5.setRange(self.j5_min, self.j5_max)
        self.slider_j6.setRange(self.j5_min, self.j6_max)

        

        #calc ik for zero position
        self.ik = self.my_chain.inverse_kinematics_frame(self.start_position, self.zero_pos, orientation_mode="all")
        self.init_position()
        #test recording
        #self.record(20,0.2)
        #self.repeat()

        #plt.ion()
        #fig, self.ax = plot_utils.init_3d_figure()
        self.fig.tight_layout()
        self.my_chain.plot(self.ik, self.ax, target=None)
        self.ax.set_xlim3d(-0.2, 0.2)
        self.ax.set_ylim3d(-0.2, 0.2)
        self.ax.set_zlim(0, 0.3)
        self.ax.view_init(20, 60)
        self.ax.set_xlabel('X [m]')
        self.ax.set_ylabel('Y [m]')
        self.ax.set_zlabel('Z [m]')
        self.ax.grid(False)
        self.ax.xaxis.pane.fill = False
        self.ax.yaxis.pane.fill = False
        self.ax.zaxis.pane.fill = False
        self.pos_error = "Positional Error: " + str(0.0)
        self.ax.set_title(self.pos_error)

    def calcInverseKinematics(self):
        self.ax.cla() 
        self.ax.set_xlim3d(-0.2, 0.2)
        self.ax.set_ylim3d(-0.2, 0.2)
        self.ax.set_zlim(0, 0.3)
        self.ax.set_xlabel('X [m]')
        self.ax.set_ylabel('Y [m]')
        self.ax.set_zlabel('Z [m]')
        self.ax.grid(False)
        self.ax.xaxis.pane.fill = False
        self.ax.yaxis.pane.fill = False
        self.ax.zaxis.pane.fill = False
        rotation_matrix = self.rotation_matrix(self.phi_pos, self.theta_pos, self.psi_pos)
        self.target_position = [[rotation_matrix[0,0],  rotation_matrix[0,1],  rotation_matrix[0,2],  self.x_pos*self.mm2m],
                                [rotation_matrix[2,0],  rotation_matrix[2,1],  rotation_matrix[2,2],  self.y_pos*self.mm2m], 
                                [rotation_matrix[1,0],  rotation_matrix[1,1],  rotation_matrix[1,2],  self.z_pos*self.mm2m], 
                                [                      0,                  0,                     0,                     1]]
        self.ik = self.my_chain.inverse_kinematics_frame(self.target_position, self.ik, orientation_mode="all", max_iter=self.ik_iterations)
        fk = self.my_chain.forward_kinematics(self.ik)
        self.pos_error = np.linalg.norm(np.subtract([self.x_pos,self.y_pos,self.z_pos], [fk[0,3]/self.mm2m,fk[1,3]/self.mm2m,fk[2,3]/self.mm2m]))
        self.pos_error = "Positional Error: " + str(np.round(self.pos_error,1))
        self.ax.set_title(self.pos_error)

        #set slider values
        self.slider_j1.setValue(int(self.ik[1]/self.deg2rad))
        self.slider_j2.setValue(int(self.ik[2]/self.deg2rad))
        self.slider_j3.setValue(int(self.ik[4]/self.deg2rad))
        self.slider_j4.setValue(int(self.ik[5]/self.deg2rad))
        self.slider_j5.setValue(int(self.ik[6]/self.deg2rad))
        self.slider_j6.setValue(int(self.ik[7]/self.deg2rad))

        #send values to robot
        if (np.abs(self.ang_1-self.ik[1])>self.rotTreshHold):
            self.ang_1 = self.ik[1]
        if (np.abs(self.ang_2-self.ik[2])>self.rotTreshHold):
            self.ang_2 = self.ik[2] - np.pi/2
        if (np.abs(self.ang_3-self.ik[4])>self.rotTreshHold):
            self.ang_3 = self.ik[4] + np.pi
        if (np.abs(self.ang_4-self.ik[5])>self.rotTreshHold):
            self.ang_4 = self.ik[5]
        if (np.abs(self.ang_5-self.ik[6])>self.rotTreshHold):
            self.ang_5 = self.ik[6]
        if (np.abs(self.ang_6-self.ik[7])>self.rotTreshHold):
            self.ang_6 = self.ik[7]
    
        msg = "," + "runToPos" + "," + str(format(round(self.ang_1/self.deg2rad,2), '.3f')) + "," + str(format(round(self.ang_2/self.deg2rad,2), '.3f')) + "," + str(format(round(self.ang_3/self.deg2rad,2), '.3f')) + "," + str(format(round(self.ang_4/self.deg2rad,2), '.3f')) + "," + str(format(round(self.ang_5/self.deg2rad,2), '.3f')) + "," + str(format(round(self.ang_6/self.deg2rad,2), '.3f')) + "," + str(format(round(self.velocity,2), '.3f')) + "," + str(format(round(self.acceleration,2), '.3f')) + ",\n"
        
        print("sent message: " + msg)
        self.ser.write((msg).encode('utf-8'))
        self.my_chain.plot(self.ik, self.ax, target=self.target_position)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def calcForwardKinematics(self):
        self.fk = self.my_chain.forward_kinematics([0.0, self.des_j1_pos*self.deg2rad, self.des_j2_pos*self.deg2rad, 0.0, self.des_j3_pos*self.deg2rad, self.des_j4_pos*self.deg2rad, self.des_j5_pos*self.deg2rad, self.des_j6_pos*self.deg2rad])

    def rotation_angles(self, matrix):
        """
        input
            matrix = 3x3 rotation matrix (numpy array)
        output
            theta1, theta2, theta3 = rotation angles in rotation order
        """
        r11, r12, r13 = matrix[0]
        r21, r22, r23 = matrix[1]
        r31, r32, r33 = matrix[2]
        theta1 = np.arctan(-r23 / r33)/self.deg2rad
        theta2 = np.arctan(r13 * np.cos(theta1) / r33)/self.deg2rad
        theta3 = np.arctan(-r12 / r11)/self.deg2rad
    
        return (theta1, theta2, theta3)
    
    def rotation_matrix(self, theta1, theta2, theta3):
        """
        input
            theta1, theta2, theta3 = rotation angles in rotation order (degrees)
        output
            3x3 rotation matrix (numpy array)
        """
        c1 = np.cos(theta1 * np.pi / 180)
        s1 = np.sin(theta1 * np.pi / 180)
        c2 = np.cos(theta2 * np.pi / 180)
        s2 = np.sin(theta2 * np.pi / 180)
        c3 = np.cos(theta3 * np.pi / 180)
        s3 = np.sin(theta3 * np.pi / 180)
        
        matrix=np.array([[c2*c3, -c2*s3, s2],
                        [c1*s3+c3*s1*s2, c1*c3-s1*s2*s3, -c2*s1],
                        [s1*s3-c1*c3*s2, c3*s1+c1*s2*s3, c1*c2]])

        return matrix
    
    def init_position(self):
        #set sliders
        self.slider_x.setValue(int(self.x_zero_pos))
        self.slider_y.setValue(int(self.y_zero_pos))
        self.slider_z.setValue(int(self.z_zero_pos))
        self.slider_phi.setValue(int(self.phi_zero_pos))
        self.slider_theta.setValue(int(self.theta_zero_pos))
        self.slider_psi.setValue(int(self.psi_zero_pos))
        self.slider_j1.setValue(int(self.ik[1]/self.deg2rad))
        self.slider_j2.setValue(int(self.ik[2]/self.deg2rad))
        self.slider_j3.setValue(int(self.ik[4]/self.deg2rad))
        self.slider_j4.setValue(int(self.ik[5]/self.deg2rad))
        self.slider_j5.setValue(int(self.ik[6]/self.deg2rad))
        self.slider_j6.setValue(int(self.ik[7]/self.deg2rad))
        #update zero position
        self.j1_zero_pos = self.ik[1]/self.deg2rad
        self.j2_zero_pos = self.ik[2]/self.deg2rad
        self.j3_zero_pos = self.ik[4]/self.deg2rad
        self.j4_zero_pos = self.ik[5]/self.deg2rad
        self.j5_zero_pos = self.ik[6]/self.deg2rad
        self.j6_zero_pos = self.ik[7]/self.deg2rad
        self.init_run = False

        #send values to robot
        self.ang_1 = self.ik[1]
        self.ang_2 = self.ik[2] - np.pi/2
        self.ang_3 = self.ik[4] + np.pi
        self.ang_4 = self.ik[5]
        self.ang_5 = self.ik[6]
        self.ang_6 = self.ik[7]

        #enable motors
        self.ser.write("enableMotors\n".encode('utf-8'))
        time.sleep(0.5) 

        msg = "," + "runToPos" + "," + str(format(round(self.ang_1/self.deg2rad,2), '.3f')) + "," + str(format(round(self.ang_2/self.deg2rad,2), '.3f')) + "," + str(format(round(self.ang_3/self.deg2rad,2), '.3f')) + "," + str(format(round(self.ang_4/self.deg2rad,2), '.3f')) + "," + str(format(round(self.ang_5/self.deg2rad,2), '.3f')) + "," + str(format(round(self.ang_6/self.deg2rad,2), '.3f')) + "," + str(format(round(self.velocity,2), '.3f')) + "," + str(format(round(self.acceleration,2), '.3f')) + ",\n"
        
        print("sent message: " + msg)
        self.ser.write((msg).encode('utf-8'))

        time.sleep(0.5)

    def connectUart(self):
        self.ser = serial.Serial(self.comPort, self.baudRate, timeout=1)
        while True:
            print("Initilizing UART communication!")
            self.ser.write("InitUart\n".encode('utf-8'))
            line = self.ser.readline().decode('utf-8').rstrip()
            if (line == "UartInitialized"):
                print("UART initialized successfully!")
                return self.ser

    def record_thread(self): 
        t1=Thread(target=self.record) 
        t1.start() 

    def repeat_thread(self):
        t1=Thread(target=self.repeat) 
        t1.start() 

    def record(self):
        self.mrLst.clear()
        #disable motors
        self.ser.write("disableMotors\n".encode('utf-8'))
        time.sleep(0.1)
        self.ser.write("startRecord\n".encode('utf-8'))
        time.sleep(0.1)
        start_time = self.current_milli_time()
        time_prev = 0
        arr = []
        v1 = 50
        v2 = 50
        v3 = 50
        v4 = 50
        v5 = 50
        v6 = 50
        a1_prev = 0
        a2_prev = 0
        a3_prev = 0
        a4_prev = 0
        a5_prev = 0
        a6_prev = 0

        while (not self.stopRecord):
            if ((self.current_milli_time() - start_time) >= self.maxRecordTime*1000):
                break
            if (self.ser.in_waiting > 0):
                    line = self.ser.readline().decode('utf-8').rstrip()
            if ((self.current_milli_time() - time_prev) > self.time_step*1000):
                self.ax.cla() 
                self.ax.set_xlim3d(-0.2, 0.2)
                self.ax.set_ylim3d(-0.2, 0.2)
                self.ax.set_zlim(0, 0.3)
                self.ax.set_xlabel('X [m]')
                self.ax.set_ylabel('Y [m]')
                self.ax.set_zlabel('Z [m]')
                self.ax.grid(False)
                self.ax.xaxis.pane.fill = False
                self.ax.yaxis.pane.fill = False
                self.ax.zaxis.pane.fill = False
                arr = line.split(',')
                if (arr[0] == 'UartInitialized'):
                    arr = ['0', '0', '0', '0', '0', '0','0', '0', '0', '0', '0', '0']
                
                #plot values
                self.my_chain.plot([0.0, float(arr[0])*self.deg2rad, float(arr[1])*self.deg2rad + np.pi/2, 0.0, float(arr[2])*self.deg2rad - np.pi, float(arr[3])*self.deg2rad, float(arr[4])*self.deg2rad, float(arr[5])*self.deg2rad], self.ax)
                self.fig.canvas.draw()
                self.fig.canvas.flush_events()

                #calc speeds
                v1 = abs((float(arr[0])-a1_prev)/(self.time_step))
                v2 = abs((float(arr[1])-a2_prev)/(self.time_step))
                v3 = abs((float(arr[2])-a3_prev)/(self.time_step))
                v4 = abs((float(arr[3])-a4_prev)/(self.time_step))
                v5 = abs((float(arr[4])-a5_prev)/(self.time_step))
                v6 = abs((float(arr[5])-a6_prev)/(self.time_step))
                a1_prev = float(arr[0])
                a2_prev = float(arr[1])
                a3_prev = float(arr[2])
                a4_prev = float(arr[3])
                a5_prev = float(arr[4])
                a6_prev = float(arr[5])
                v_max = max([v1,v2,v3,v4,v5,v6])/self.slowFactor
                if (v_max>self.vmax):
                    self.vmax = v_max

                arr.append((self.current_milli_time() - time_prev)) #save duration of the step
                self.mrLst.append(arr)
                time_prev = self.current_milli_time()

        self.mrLst.pop(0)
        self.ser.write("stopRecord\n".encode('utf-8'))
        if (self.vmax> self.velocity):
            self.vmax =self.velocity

        #enable motors
        self.ser.write("enableMotors\n".encode('utf-8'))

        #return to home position
        a1 = float(arr[0]) * self.deg2rad
        a2 = -float(arr[1]) * self.deg2rad
        a3 = -float(arr[2]) * self.deg2rad
        a4 = float(arr[3]) * self.deg2rad
        a5 = -float(arr[4]) * self.deg2rad
        a6 = float(arr[5]) * self.deg2rad
        msg = "," + "runToPos" + "," + str(format(round(a1/self.deg2rad,2), '.3f')) + "," + str(format(round(a2/self.deg2rad,2), '.3f')) + "," + str(format(round(a3/self.deg2rad,2), '.3f')) + "," + str(format(round(a4/self.deg2rad,2), '.3f')) + "," + str(format(round(a5/self.deg2rad,2), '.3f')) + "," + str(format(round(a6/self.deg2rad,2), '.3f')) + "," + str(format(round(self.velocity,2), '.3f')) + "," + str(format(round(self.acceleration,2), '.3f')) + ",\n"

        time.sleep(0.1)
        self.ser.write((msg).encode('utf-8'))
        self.ax.cla() 
        self.ax.set_xlim3d(-0.2, 0.2)
        self.ax.set_ylim3d(-0.2, 0.2)
        self.ax.set_zlim(0, 0.3)
        self.ax.set_xlabel('X [m]')
        self.ax.set_ylabel('Y [m]')
        self.ax.set_zlabel('Z [m]')
        self.ax.grid(False)
        self.ax.xaxis.pane.fill = False
        self.ax.yaxis.pane.fill = False
        self.ax.zaxis.pane.fill = False
        self.my_chain.plot([0.0, 0, np.pi/2, 0.0,  -np.pi, 0, 0, 0], self.ax)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

        #set home position    
        while(True):
            self.ser.write("setHome\n".encode('utf-8'))
            line = self.ser.readline().decode('utf-8').rstrip()
            if (line == "homeSet"):
                break

    def repeat(self):
        time_prev = self.current_milli_time()
        counter = 0

        for i in self.mrLst:
            self.ax.cla() 
            self.ax.set_xlim3d(-0.2, 0.2)
            self.ax.set_ylim3d(-0.2, 0.2)
            self.ax.set_zlim(0, 0.3)
            self.ax.set_xlabel('X [m]')
            self.ax.set_ylabel('Y [m]')
            self.ax.set_zlabel('Z [m]')
            self.ax.grid(False)
            self.ax.xaxis.pane.fill = False
            self.ax.yaxis.pane.fill = False
            self.ax.zaxis.pane.fill = False
            a1 = -float(i[0])
            a2 = float(i[1])
            a3 = float(i[2])
            a4 = -float(i[3])
            a5 = float(i[4])
            a6 = -float(i[5])
            newTimeStep = i[12]
            msg = "," + "runToPos" + "," + str(format(round(a1,2), '.3f')) + "," + str(format(round(a2,2), '.3f')) + "," + str(format(round(a3,2), '.3f')) + "," + str(format(round(a4,2), '.3f')) + "," + str(format(round(a5,2), '.3f')) + "," + str(format(round(a6,2), '.3f')) + "," + str(format(round(self.vmax,2), '.3f')) + "," + str(format(round(self.acceleration,2), '.3f')) + ",\n"
            #msg = "," + "runSeq" + "," + str(format(round(a1,2), '.3f')) + "," + str(format(round(a2,2), '.3f')) + "," + str(format(round(a3,2), '.3f')) + "," + str(format(round(a4,2), '.3f')) + "," + str(format(round(a5,2), '.3f')) + "," + str(format(round(a6,2), '.3f')) + "," + str(format(round(v1,2), '.3f')) + "," + str(format(round(v2,2), '.3f')) + "," + str(format(round(v3,2), '.3f')) + "," + str(format(round(v4,2), '.3f')) + "," + str(format(round(v5,2), '.3f')) + "," + str(format(round(v6,2), '.3f')) + ",\n"
            self.ser.write((msg).encode('utf-8'))
            while(not self.stopRepeat):
                    if ((self.current_milli_time()-time_prev)>newTimeStep*(self.slowFactor+0.2)):
                        time_prev = self.current_milli_time()
                        self.my_chain.plot([0.0, a1 * self.deg2rad, a2 * self.deg2rad + np.pi/2, 0.0, a3 * self.deg2rad - np.pi, a4 * self.deg2rad, a5 * self.deg2rad, a6 * self.deg2rad], self.ax)
                        self.fig.canvas.draw()
                        self.fig.canvas.flush_events()
                        break
            counter = counter + 1
            if (self.stopRepeat):
                counter == len(self.mrLst)

            #return home if its the last position
            if (counter == len(self.mrLst)):
                a1 = 0
                a2 = 0
                a3 = 0
                a4 = 0
                a5 = 0
                a6 = 0
                msg = "," + "runToPos" + "," + str(format(round(a1/self.deg2rad,2), '.3f')) + "," + str(format(round(a2/self.deg2rad,2), '.3f')) + "," + str(format(round(a3/self.deg2rad,2), '.3f')) + "," + str(format(round(a4/self.deg2rad,2), '.3f')) + "," + str(format(round(a5/self.deg2rad,2), '.3f')) + "," + str(format(round(a6/self.deg2rad,2), '.3f')) + "," + str(format(round(self.velocity,2), '.3f')) + "," + str(format(round(self.acceleration,2), '.3f')) + ",\n"
                self.ser.write((msg).encode('utf-8'))
                break
        
        self.ax.cla() 
        self.ax.set_xlim3d(-0.2, 0.2)
        self.ax.set_ylim3d(-0.2, 0.2)
        self.ax.set_zlim(0, 0.3)
        self.ax.set_xlabel('X [m]')
        self.ax.set_ylabel('Y [m]')
        self.ax.set_zlabel('Z [m]')
        self.ax.grid(False)
        self.ax.xaxis.pane.fill = False
        self.ax.yaxis.pane.fill = False
        self.ax.zaxis.pane.fill = False
        self.my_chain.plot([0.0, 0, np.pi/2, 0.0,  -np.pi, 0, 0, 0], self.ax)
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    
    def current_milli_time(self):
        return round(time.time() * 1000)
    

if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = Window()
    win.show()

    sys.exit(app.exec())
    