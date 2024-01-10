import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from ament_index_python.packages import get_package_share_directory

import os
import time
import math
import smbus
import copy
import threading
import numpy as np
from mpu6050 import mpu6050


class DogCommands(Node):
    def __init__(self):
        super().__init__('dog_commands')
        self.subscription = self.create_subscription(Twist, '/spot_go', self.dog_callback, 10)
        self.subscription = self.create_subscription(Joy,'/joy',self.joy_callback,10)

        self.body_heigth = 0
        self.speed = 8

    def dog_callback(self, msg):
        x = msg.linear.x
        y = msg.angular.z
        
        if x >= 0.3:
            control.forWard()
        if x <= -0.3:
            control.backWard()
        if y >=0.3:
            control.turnLeft()
        if y <= -0.3:
            control.turnRight()
        if x == 0.0 and y == 0.0:
            control.relax()
        #else:
        #    control.stop()

    def joy_callback(self, msg):
        # Check if the message has enough buttons
#        if len(msg.buttons) >= 4:
#            button_A = msg.buttons[0]
#            button_B = msg.buttons[1]
#            button_X = msg.buttons[2]
#            button_Y = msg.buttons[3]

            # Handle button presses and publish corresponding values on /LEDs topic
#            if button_A == 1:
#                self.case_(1)
#            elif button_B == 1:
#                self.case_(0)
#            elif button_X == 1:
#                self.case_(2)
#            elif button_Y == 1:
#                self.case_(3)
            
            axis_3_value = msg.axes[3]
            axis_4_value = msg.axes[4]

            if axis_3_value >= 0.3:
                control.setpLeft()
            if axis_3_value <= -0.3:
                control.setpRight()



            # Handle servo control based on axes 6 and 7
            axis_6_value = msg.axes[6]
            axis_7_value = msg.axes[7]

            if axis_6_value == 1:
                self.speed = min(self.speed + 1, 10)
                self.get_logger().info("Speed beträgt %d " %self.speed)
            elif axis_6_value == -1:
                self.speed = max(self.speed - 1, 0)
                self.get_logger().info("Speed beträgt %d " %self.speed)

            if axis_7_value == 1:
                self.body_heigth = min(self.body_heigth - 1, 150)
                control.upAndDown(self.body_heigth)
                self.get_logger().info("Körper Höhe beträgt %d " %self.body_heigth)
            elif axis_7_value == -1:
                self.body_heigth = max(self.body_heigth + 1, 0)
                control.upAndDown(self.body_heigth)
                self.get_logger().info("Körper Höhe beträgt %d " %self.body_heigth)


class COMMAND:
    CMD_MOVE_STOP = "CMD_MOVE_STOP"
    CMD_MOVE_FORWARD = "CMD_MOVE_FORWARD" 
    CMD_MOVE_BACKWARD = "CMD_MOVE_BACKWARD"
    CMD_MOVE_LEFT = "CMD_MOVE_LEFT"
    CMD_MOVE_RIGHT = "CMD_MOVE_RIGHT"
    CMD_TURN_LEFT = "CMD_TURN_LEFT"
    CMD_TURN_RIGHT = "CMD_TURN_RIGHT"
    CMD_BUZZER = "CMD_BUZZER"
    CMD_LED_MOD = "CMD_LED_MOD"
    CMD_LED = "CMD_LED"
    CMD_BALANCE = "CMD_BALANCE"
    CMD_SONIC = "CMD_SONIC"
    CMD_HEIGHT = "CMD_HEIGHT"
    CMD_HORIZON = "CMD_HORIZON"
    CMD_HEAD = "CMD_HEAD"
    CMD_CALIBRATION = "CMD_CALIBRATION"
    CMD_POWER = "CMD_POWER"
    CMD_ATTITUDE = "CMD_ATTITUDE"
    CMD_RELAX = "CMD_RELAX"
    CMD_WORKING_TIME = "CMD_WORKING_TIME"
    

cmd = COMMAND()


class PCA9685:

  # Registers/etc.
  __SUBADR1            = 0x02
  __SUBADR2            = 0x03
  __SUBADR3            = 0x04
  __MODE1              = 0x00
  __PRESCALE           = 0xFE
  __LED0_ON_L          = 0x06
  __LED0_ON_H          = 0x07
  __LED0_OFF_L         = 0x08
  __LED0_OFF_H         = 0x09
  __ALLLED_ON_L        = 0xFA
  __ALLLED_ON_H        = 0xFB
  __ALLLED_OFF_L       = 0xFC
  __ALLLED_OFF_H       = 0xFD

  def __init__(self, address=0x40, debug=False):
    self.bus = smbus.SMBus(1)
    self.address = address
    self.debug = debug
    self.write(self.__MODE1, 0x00)
    
  def write(self, reg, value):
    "Writes an 8-bit value to the specified register/address"
    self.bus.write_byte_data(self.address, reg, value)
      
  def read(self, reg):
    "Read an unsigned byte from the I2C device"
    result = self.bus.read_byte_data(self.address, reg)
    return result
    
  def setPWMFreq(self, freq):
    "Sets the PWM frequency"
    prescaleval = 25000000.0    # 25MHz
    prescaleval /= 4096.0       # 12-bit
    prescaleval /= float(freq)
    prescaleval -= 1.0
    prescale = math.floor(prescaleval + 0.5)


    oldmode = self.read(self.__MODE1);
    newmode = (oldmode & 0x7F) | 0x10        # sleep
    self.write(self.__MODE1, newmode)        # go to sleep
    self.write(self.__PRESCALE, int(math.floor(prescale)))
    self.write(self.__MODE1, oldmode)
    time.sleep(0.005)
    self.write(self.__MODE1, oldmode | 0x80)

  def setPWM(self, channel, on, off):
    "Sets a single PWM channel"
    self.write(self.__LED0_ON_L+4*channel, on & 0xFF)
    self.write(self.__LED0_ON_H+4*channel, on >> 8)
    self.write(self.__LED0_OFF_L+4*channel, off & 0xFF)
    self.write(self.__LED0_OFF_H+4*channel, off >> 8)
  def setMotorPwm(self,channel,duty):
    self.setPWM(channel,0,duty)
  def setServoPulse(self, channel, pulse):
    "Sets the Servo Pulse,The PWM frequency must be 50HZ"
    pulse = pulse*4096/20000        #PWM frequency is 50HZ,the period is 20000us
    self.setPWM(channel, 0, int(pulse))

class Servo:
    def __init__(self):
        self.angleMin=18
        self.angleMax=162
        self.pwm = PCA9685(address=0x40, debug=True)   
        self.pwm.setPWMFreq(50)               # Set the cycle frequency of PWM
    #Convert the input angle to the value of pca9685
    def map(self,value,fromLow,fromHigh,toLow,toHigh):
        return (toHigh-toLow)*(value-fromLow) / (fromHigh-fromLow) + toLow
    def setServoAngle(self,channel, angle):
        if angle < self.angleMin:
            angle = self.angleMin
        elif angle >self.angleMax:
            angle=self.angleMax
        date=self.map(angle,0,180,102,512)
        #print(date,date/4096*0.02)
        self.pwm.setPWM(channel, 0, int(date))

class Incremental_PID:
    ''' PID controller'''
    def __init__(self,P=0.0,I=0.0,D=0.0):
        self.setPoint = 0.0
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.last_error = 0.0
        self.P_error = 0.0
        self.I_error = 0.0
        self.D_error = 0.0
        self.I_saturation = 10.0
        self.output = 0.0

    def PID_compute(self,feedback_val):
        error = self.setPoint - feedback_val
        self.P_error = self.Kp * error
        self.I_error += error 
        self.D_error = self.Kd * (error - self.last_error)
        if (self.I_error < -self.I_saturation ):
            self.I_error = -self.I_saturation
        elif (self.I_error > self.I_saturation):
            self.I_error = self.I_saturation
        self.output = self.P_error + (self.Ki * self.I_error) + self.D_error
        self.last_error = error
        return self.output

    def setKp(self,proportional_gain):
        self.Kp = proportional_gain

    def setKi(self,integral_gain):
        self.Ki = integral_gain

    def setKd(self,derivative_gain):
        self.Kd = derivative_gain

    def setI_saturation(self,saturation_val):
        self.I_saturation = saturation_val

class Kalman_filter:
    def __init__(self,Q,R):
        self.Q = Q
        self.R = R
        self.P_k_k1 = 1
        self.Kg = 0
        self.P_k1_k1 = 1
        self.x_k_k1 = 0
        self.ADC_OLD_Value = 0
        self.Z_k = 0
        self.kalman_adc_old=0
        
    def kalman(self,ADC_Value):
        self.Z_k = ADC_Value
        if (abs(self.kalman_adc_old-ADC_Value)>=60):
            self.x_k1_k1= ADC_Value*0.400 + self.kalman_adc_old*0.600
        else:
            self.x_k1_k1 = self.kalman_adc_old;
        self.x_k_k1 = self.x_k1_k1
        self.P_k_k1 = self.P_k1_k1 + self.Q
        self.Kg = self.P_k_k1/(self.P_k_k1 + self.R)
        kalman_adc = self.x_k_k1 + self.Kg * (self.Z_k - self.kalman_adc_old)
        self.P_k1_k1 = (1 - self.Kg)*self.P_k_k1
        self.P_k_k1 = self.P_k1_k1
        self.kalman_adc_old = kalman_adc
        return kalman_adc

class IMU:
    def __init__(self):
        self.Kp = 100 
        self.Ki = 0.002 
        self.halfT = 0.001 

        self.q0 = 1
        self.q1 = 0
        self.q2 = 0
        self.q3 = 0

        self.exInt = 0
        self.eyInt = 0
        self.ezInt = 0
        self.pitch = 0
        self.roll =0
        self.yaw = 0
        
        self.sensor = mpu6050(address=0x68)                    
        self.sensor.set_accel_range(mpu6050.ACCEL_RANGE_2G)   
        self.sensor.set_gyro_range(mpu6050.GYRO_RANGE_250DEG)  
        
        self.kalman_filter_AX =  Kalman_filter(0.001,0.1)
        self.kalman_filter_AY =  Kalman_filter(0.001,0.1)
        self.kalman_filter_AZ =  Kalman_filter(0.001,0.1)

        self.kalman_filter_GX =  Kalman_filter(0.001,0.1)
        self.kalman_filter_GY =  Kalman_filter(0.001,0.1)
        self.kalman_filter_GZ =  Kalman_filter(0.001,0.1)
        
        self.Error_value_accel_data,self.Error_value_gyro_data=self.average_filter()
    
    def average_filter(self):
        sum_accel_x=0
        sum_accel_y=0
        sum_accel_z=0
        
        sum_gyro_x=0
        sum_gyro_y=0
        sum_gyro_z=0
        for i in range(100):
            accel_data = self.sensor.get_accel_data()   
            gyro_data = self.sensor.get_gyro_data()      
            
            sum_accel_x+=accel_data['x']
            sum_accel_y+=accel_data['y']
            sum_accel_z+=accel_data['z']
            
            sum_gyro_x+=gyro_data['x']
            sum_gyro_y+=gyro_data['y']
            sum_gyro_z+=gyro_data['z']
            
        sum_accel_x/=100
        sum_accel_y/=100
        sum_accel_z/=100
        
        sum_gyro_x/=100
        sum_gyro_y/=100
        sum_gyro_z/=100
        
        accel_data['x']=sum_accel_x
        accel_data['y']=sum_accel_y
        accel_data['z']=sum_accel_z-9.8
        
        gyro_data['x']=sum_gyro_x
        gyro_data['y']=sum_gyro_y
        gyro_data['z']=sum_gyro_z
        
        return accel_data,gyro_data
    
    def imuUpdate(self):
        accel_data = self.sensor.get_accel_data()    
        gyro_data = self.sensor.get_gyro_data() 
        ax=self.kalman_filter_AX.kalman(accel_data['x']-self.Error_value_accel_data['x'])
        ay=self.kalman_filter_AY.kalman(accel_data['y']-self.Error_value_accel_data['y'])
        az=self.kalman_filter_AZ.kalman(accel_data['z']-self.Error_value_accel_data['z'])
        gx=self.kalman_filter_GX.kalman(gyro_data['x']-self.Error_value_gyro_data['x'])
        gy=self.kalman_filter_GY.kalman(gyro_data['y']-self.Error_value_gyro_data['y'])
        gz=self.kalman_filter_GZ.kalman(gyro_data['z']-self.Error_value_gyro_data['z'])

        norm = math.sqrt(ax*ax+ay*ay+az*az)
        
        ax = ax/norm
        ay = ay/norm
        az = az/norm
        
        vx = 2*(self.q1*self.q3 - self.q0*self.q2)
        vy = 2*(self.q0*self.q1 + self.q2*self.q3)
        vz = self.q0*self.q0 - self.q1*self.q1 - self.q2*self.q2 + self.q3*self.q3
        
        ex = (ay*vz - az*vy)
        ey = (az*vx - ax*vz)
        ez = (ax*vy - ay*vx)
        
        self.exInt += ex*self.Ki
        self.eyInt += ey*self.Ki
        self.ezInt += ez*self.Ki
        
        gx += self.Kp*ex + self.exInt
        gy += self.Kp*ey + self.eyInt
        gz += self.Kp*ez + self.ezInt
        
        self.q0 += (-self.q1*gx - self.q2*gy - self.q3*gz)*self.halfT
        self.q1 += (self.q0*gx + self.q2*gz - self.q3*gy)*self.halfT
        self.q2 += (self.q0*gy - self.q1*gz + self.q3*gx)*self.halfT
        self.q3 += (self.q0*gz + self.q1*gy - self.q2*gx)*self.halfT
        
        norm = math.sqrt(self.q0*self.q0 + self.q1*self.q1 + self.q2*self.q2 + self.q3*self.q3)
        self.q0 /= norm
        self.q1 /= norm
        self.q2 /= norm
        self.q3 /= norm
        
        pitch = math.asin(-2*self.q1*self.q3+2*self.q0*self.q2)*57.3
        roll = math.atan2(2*self.q2*self.q3+2*self.q0*self.q1,-2*self.q1*self.q1-2*self.q2*self.q2+1)*57.3
        yaw = math.atan2(2*(self.q1*self.q2 + self.q0*self.q3),self.q0*self.q0+self.q1*self.q1-self.q2*self.q2-self.q3*self.q3)*57.3
        self.pitch = pitch
        self.roll =roll
        self.yaw = yaw
        return self.pitch,self.roll,self.yaw

class Control:   

    def __init__(self):
        self.imu=IMU()
        self.servo=Servo()
        self.pid = Incremental_PID(0.5,0.0,0.0025)
        self.speed = 6
        self.height = 120    # spot : 200      default : 99
        self.timeout = 0
        self.move_flag = 0
        self.move_count = 0
        self.move_timeout = 0
        self.order = ['','','','','']
        self.point = [[0, 99, 10], [0, 99, 10], [0, 99, -10], [0, 99, -10]]
        self.points = os.path.join(get_package_share_directory("spot_rc"), "spot_rc", "point.txt")
        self.calibration_point = self.readFromTxt(self.points)
        self.angle = [[90,0,0],[90,0,0],[90,0,0],[90,0,0]]
        self.calibration_angle=[[0,0,0],[0,0,0],[0,0,0],[0,0,0]]
        self.relax_flag=True
        self.balance_flag=False
        self.attitude_flag=False
        self.Thread_conditiona=threading.Thread(target=self.condition)
        self.calibration()
        self.relax(True)
    def readFromTxt(self,filename):
        file1 = open(self.points) #open(filename + ".txt", "r")
        list_row = file1.readlines()
        list_source = []
        for i in range(len(list_row)):
            column_list = list_row[i].strip().split("\t")
            list_source.append(column_list)
        for i in range(len(list_source)):
            for j in range(len(list_source[i])):
                list_source[i][j] = int(list_source[i][j])
        file1.close()
        return list_source

    def saveToTxt(self,list, filename):
        file2 = open(self.points) #open(filename + '.txt', 'w')
        for i in range(len(list)):
            for j in range(len(list[i])):
                file2.write(str(list[i][j]))
                file2.write('\t')
            file2.write('\n')
        file2.close()
        
    def coordinateToAngle(self,x,y,z,l1=23,l2=55,l3=55):     # 23  55  55  // 10 111 120  l1=10,l2=111,l3=120
        a=math.pi/2-math.atan2(z,y)
        x_3=0
        x_4=l1*math.sin(a)
        x_5=l1*math.cos(a)
        l23=math.sqrt((z-x_5)**2+(y-x_4)**2+(x-x_3)**2)
        w=(x-x_3)/l23
        v=(l2*l2+l23*l23-l3*l3)/(2*l2*l23)
        b=math.asin(round(w,2))-math.acos(round(v,2))
        c=math.pi-math.acos(round((l2**2+l3**2-l23**2)/(2*l3*l2),2))
        a=round(math.degrees(a))
        b=round(math.degrees(b))
        c=round(math.degrees(c))
        return a,b,c
    
    def angleToCoordinate(self,a,b,c,l1=23,l2=55,l3=55):     # 23  55  55 l1=23,l2=55,l3=55
        a=math.pi/180*a
        b=math.pi/180*b
        c=math.pi/180*c
        x=l3*math.sin(b+c)+l2*math.sin(b)
        y=l3*math.sin(a)*math.cos(b+c)+l2*math.sin(a)*math.cos(b)+l1*math.sin(a)
        z=l3*math.cos(a)*math.cos(b+c)+l2*math.cos(a)*math.cos(b)+l1*math.cos(a)
        return x,y,z
    
    def calibration(self):
        for i in range(4):
            self.calibration_angle[i][0],self.calibration_angle[i][1],self.calibration_angle[i][2]=self.coordinateToAngle(self.calibration_point[i][0],
                                                                                                                          self.calibration_point[i][1],
                                                                                                                          self.calibration_point[i][2])
        for i in range(4):
            self.angle[i][0],self.angle[i][1],self.angle[i][2]=self.coordinateToAngle(self.point[i][0],
                                                                                      self.point[i][1],
                                                                                      self.point[i][2])
        for i in range(4):
            self.calibration_angle[i][0]=self.calibration_angle[i][0]-self.angle[i][0]
            self.calibration_angle[i][1]=self.calibration_angle[i][1]-self.angle[i][1]
            self.calibration_angle[i][2]=self.calibration_angle[i][2]-self.angle[i][2]
    def run(self):
        if self.checkPoint():
            try:
                for i in range(4):
                    self.angle[i][0],self.angle[i][1],self.angle[i][2]=self.coordinateToAngle(self.point[i][0],
                                                                                              self.point[i][1],
                                                                                              self.point[i][2])
                for i in range(2):
                    self.angle[i][0]=self.restriction(self.angle[i][0]+self.calibration_angle[i][0],0,180)
                    self.angle[i][1]=self.restriction(90-(self.angle[i][1]+self.calibration_angle[i][1]),0,180)
                    self.angle[i][2]=self.restriction(self.angle[i][2]+self.calibration_angle[i][2],0,180)
                    self.angle[i+2][0]=self.restriction(self.angle[i+2][0]+self.calibration_angle[i+2][0],0,180)
                    self.angle[i+2][1]=self.restriction(90+self.angle[i+2][1]+self.calibration_angle[i+2][1],0,180)
                    self.angle[i+2][2]=self.restriction(180-(self.angle[i+2][2]+self.calibration_angle[i+2][2]),0,180)
                for i in range(2):
                    self.servo.setServoAngle(4+i*3,self.angle[i][0])
                    self.servo.setServoAngle(3+i*3,self.angle[i][1])
                    self.servo.setServoAngle(2+i*3,self.angle[i][2])
                    self.servo.setServoAngle(8+i*3,self.angle[i+2][0])
                    self.servo.setServoAngle(9+i*3,self.angle[i+2][1])
                    self.servo.setServoAngle(10+i*3,self.angle[i+2][2])
            except Exception as e:
                pass
        else:
            print("This coordinate point is out of the active range")
            print(self.angle[0][0],self.angle[0][1],self.angle[0][2])
            print(self.angle[1][0],self.angle[1][1],self.angle[1][2])
            print(self.angle[2][0],self.angle[2][1],self.angle[2][2])
            print(self.angle[3][0],self.angle[3][1],self.angle[3][2])
            
    def checkPoint(self):
        flag=True
        leg_lenght=[0,0,0,0,0,0]  
        for i in range(4):
          leg_lenght[i]=math.sqrt(self.point[i][0]**2+self.point[i][1]**2+self.point[i][2]**2)
        for i in range(4         ):
          if leg_lenght[i] > 150 or leg_lenght[i] < 25:    #  <--- ?
            flag=False
        return flag
            
    def condition(self):
        while True:
            try:
                if time.time()-self.move_timeout > 60 and self.move_timeout!=0 and self.relax_flag==True:
                    self.move_count=0
                    self.move_timeout=time.time()
                if self.move_count < 180:
                    if (time.time()-self.timeout)>10 and self.timeout!=0 and self.relax_flag==False and self.order[0] == '':
                        self.timeout=time.time()
                        self.relax_flag=True
                        self.relax(True)
                        self.order=['','','','','']
                    if self.relax_flag==True and self.order[0] != ''  and self.order[0] !=cmd.CMD_RELAX: 
                        self.relax(False)
                        self.relax_flag=False
                    if self.attitude_flag==True and self.order[0] !=cmd.CMD_ATTITUDE and self.order[0] != '':
                        self.stop()   
                        self.attitude_flag=False  
                    if self.relax_flag==False: 
                        self.move_count+=time.time()-self.move_timeout
                        self.move_timeout=time.time()
                    if self.order[0]==cmd.CMD_MOVE_STOP:
                        self.order=['','','','','']
                        self.stop()
                    elif self.order[0]==cmd.CMD_MOVE_FORWARD:
                        self.speed=int(self.order[1])
                        self.forWard()
                    elif self.order[0]==cmd.CMD_MOVE_BACKWARD:
                        self.speed=int(self.order[1])
                        self.backWard()
                    elif self.order[0]==cmd.CMD_MOVE_LEFT:
                        self.speed=int(self.order[1])
                        self.setpLeft()
                    elif self.order[0]==cmd.CMD_MOVE_RIGHT:
                        self.speed=int(self.order[1])
                        self.setpRight()
                    elif self.order[0]==cmd.CMD_TURN_LEFT:
                        self.speed=int(self.order[1])
                        self.turnLeft()
                    elif self.order[0]==cmd.CMD_TURN_RIGHT:
                        self.speed=int(self.order[1])
                        self.turnRight()
                    elif self.order[0]==cmd.CMD_RELAX:
                        if self.relax_flag:
                            self.relax_flag=False
                            self.relax(False)
                        else:
                            self.relax_flag=True
                            self.relax(True)
                        self.order=['','','','','']
                    elif self.order[0]==cmd.CMD_HEIGHT:
                        self.upAndDown(int(self.order[1]))
                        self.order=['','','','','']
                    elif self.order[0]==cmd.CMD_HORIZON:
                        self.beforeAndAfter(int(self.order[1]))
                        self.order=['','','','','']
                    elif self.order[0]==cmd.CMD_ATTITUDE:
                        self.attitude_flag=True
                        self.attitude(self.order[1],self.order[2],self.order[3])
                    elif self.order[0]==cmd.CMD_CALIBRATION:
                        self.move_count=0
                        if self.order[1]=="one":
                            self.calibration_point[0][0]=int(self.order[2])
                            self.calibration_point[0][1]=int(self.order[3])
                            self.calibration_point[0][2]=int(self.order[4])
                            self.calibration()
                            self.run()
                        elif self.order[1]=="two":
                            self.calibration_point[1][0]=int(self.order[2])
                            self.calibration_point[1][1]=int(self.order[3])
                            self.calibration_point[1][2]=int(self.order[4])
                            self.calibration()
                            self.run()
                        elif self.order[1]=="three":
                            self.calibration_point[2][0]=int(self.order[2])
                            self.calibration_point[2][1]=int(self.order[3])
                            self.calibration_point[2][2]=int(self.order[4])
                            self.calibration()
                            self.run()   
                        elif self.order[1]=="four":
                            self.calibration_point[3][0]=int(self.order[2])
                            self.calibration_point[3][1]=int(self.order[3])
                            self.calibration_point[3][2]=int(self.order[4])
                            self.calibration()
                            self.run()
                        elif self.order[1]=="save":
                            self.saveToTxt(self.calibration_point,'point')
                            self.stop()
                    elif self.order[0]==cmd.CMD_BALANCE and self.order[1]=='1':
                        Thread_IMU=threading.Thread(target=self.IMU6050())
                        Thread_IMU.start()
                        break
                elif self.move_count > 180 :
                    self.relax_flag=True
                    self.relax(True)
                    if self.move_flag!=1:
                        self.move_flag=1
                    if  self.move_count > 240:
                        self.move_count=0
                        self.move_flag=0
                    self.order=['','','','','']
            except Exception as e:
                print(e)
    def restriction(self,var,v_min,v_max):
        if var < v_min:
            return v_min
        elif var > v_max:
            return v_max
        else:
            return var            
    def map(self,value,fromLow,fromHigh,toLow,toHigh):
        return (toHigh-toLow)*(value-fromLow) / (fromHigh-fromLow) + toLow
    def changeCoordinates(self,move_order,X1=0,Y1=96,Z1=0,X2=0,Y2=96,Z2=0,pos=np.mat(np.zeros((3, 4)))): # Y1 & Y2 = 96  -- Spot 220
        if move_order == 'turnLeft':  
            for i in range(2):
                self.point[2*i][0]=((-1)**(1+i))*X1+10
                self.point[2*i][1]=Y1
                self.point[2*i][2]=((-1)**(i))*Z1+((-1)**i)*10
                self.point[1+2*i][0]=((-1)**(1+i))*X2+10
                self.point[1+2*i][1]=Y2
                self.point[1+2*i][2]=((-1)**(1+i))*Z2+((-1)**i)*10
        elif move_order == 'turnRight': 
            for i in range(2):
                self.point[2*i][0]=((-1)**(i))*X1+10
                self.point[2*i][1]=Y1
                self.point[2*i][2]=((-1)**(1+i))*Z1+((-1)**i)*10
                self.point[1+2*i][0]=((-1)**(i))*X2+10
                self.point[1+2*i][1]=Y2
                self.point[1+2*i][2]=((-1)**(i))*Z2+((-1)**i)*10
        elif (move_order == 'height') or (move_order == 'horizon'):   
            for i in range(2):
                self.point[3*i][0]=X1+10
                self.point[3*i][1]=Y1
                self.point[1+i][0]=X2+10
                self.point[1+i][1]=Y2
        elif move_order == 'Attitude Angle': 
            for i in range(2):
                self.point[3-i][0]=pos[0,1+2*i]+10
                self.point[3-i][1]=pos[2,1+2*i]
                self.point[3-i][2]=pos[1,1+2*i]      
                self.point[i][0]=pos[0,2*i]+10
                self.point[i][1]=pos[2,2*i]
                self.point[i][2]=pos[1,2*i]
        else: #'backWard' 'forWard' 'setpRight' 'setpLeft'
            for i in range(2):
                self.point[i*2][0]=X1+10
                self.point[i*2][1]=Y1
                self.point[i*2+1][0]=X2+10
                self.point[i*2+1][1]=Y2
                self.point[i*2][2]=Z1+((-1)**i)*10
                self.point[i*2+1][2]=Z2+((-1)**i)*10
        self.run()
    def backWard(self):
        for i in range(450,89,-self.speed):
            X1=12*math.cos(i*math.pi/180)
            Y1=6*math.sin(i*math.pi/180)+self.height
            X2=12*math.cos((i+180)*math.pi/180)
            Y2=6*math.sin((i+180)*math.pi/180)+self.height
            if Y2 > self.height:
                Y2=self.height
            if Y1 > self.height:
                Y1=self.height
            self.changeCoordinates('backWard',X1,Y1,0,X2,Y2,0)
            #time.sleep(0.01)
    def forWard(self):
        for i in range(90,451,self.speed):
            X1=12*math.cos(i*math.pi/180)
            Y1=6*math.sin(i*math.pi/180)+self.height
            X2=12*math.cos((i+180)*math.pi/180)
            Y2=6*math.sin((i+180)*math.pi/180)+self.height
            if Y2 > self.height:
                Y2=self.height
            if Y1 > self.height:
                Y1=self.height
            self.changeCoordinates('forWard',X1,Y1,0,X2,Y2,0)
            #time.sleep(0.01)
    def turnLeft(self):
        for i in range(0,361,self.speed):
            X1=3*math.cos(i*math.pi/180)
            Y1=8*math.sin(i*math.pi/180)+self.height
            X2=3*math.cos((i+180)*math.pi/180)
            Y2=8*math.sin((i+180)*math.pi/180)+self.height
            if Y2 > self.height:
                Y2=self.height
            if Y1 > self.height:
                Y1=self.height
            Z1=X1
            Z2=X2
            self.changeCoordinates('turnLeft',X1,Y1,Z1,X2,Y2,Z2)
            #time.sleep(0.01)
    
    def turnRight(self):
         for i in range(0,361,self.speed):
            X1=3*math.cos(i*math.pi/180)
            Y1=8*math.sin(i*math.pi/180)+self.height
            X2=3*math.cos((i+180)*math.pi/180)
            Y2=8*math.sin((i+180)*math.pi/180)+self.height
            if Y2 > self.height:
                Y2=self.height
            if Y1 > self.height:
                Y1=self.height
            Z1=X1
            Z2=X2
            self.changeCoordinates('turnRight',X1,Y1,Z1,X2,Y2,Z2)  
            #time.sleep(0.01)
    def stop(self):
        p=[[10, self.height, 10], [10, self.height, 10], [10, self.height, -10], [10, self.height, -10]]
        for i in range(4):
            p[i][0]=(p[i][0]-self.point[i][0])/50
            p[i][1]=(p[i][1]-self.point[i][1])/50
            p[i][2]=(p[i][2]-self.point[i][2])/50
        for j in range(50):
            for i in range(4):
                self.point[i][0]+=p[i][0]
                self.point[i][1]+=p[i][1]
                self.point[i][2]+=p[i][2]
            self.run()
    def setpLeft(self):
        for i in range(90,451,self.speed):
            Z1=10*math.cos(i*math.pi/180)
            Y1=5*math.sin(i*math.pi/180)+self.height
            Z2=10*math.cos((i+180)*math.pi/180)
            Y2=5*math.sin((i+180)*math.pi/180)+self.height
            if Y1 > self.height:
                Y1=self.height
            if Y2 > self.height:
                Y2=self.height
            self.changeCoordinates('setpLeft',0,Y1,Z1,0,Y2,Z2)
            #time.sleep(0.01)
    def setpRight(self):
        for i in range(450,89,-self.speed):
            Z1=10*math.cos(i*math.pi/180)
            Y1=5*math.sin(i*math.pi/180)+self.height
            Z2=10*math.cos((i+180)*math.pi/180)
            Y2=5*math.sin((i+180)*math.pi/180)+self.height
            if Y1 > self.height:
                Y1=self.height
            if Y2 > self.height:
                Y2=self.height
            self.changeCoordinates('setpRight',0,Y1,Z1,0,Y2,Z2)
            #time.sleep(0.01)
    def relax(self,flag=False):
        if flag==True:
            p=[[55, 78, 0], [55, 78, 0], [55, 78, 0], [55, 78, 0]]
            for i in range(4):
                p[i][0]=(self.point[i][0]-p[i][0])/50
                p[i][1]=(self.point[i][1]-p[i][1])/50
                p[i][2]=(self.point[i][2]-p[i][2])/50
            for j in range(1,51):
                for i in range(4):
                    self.point[i][0]-=p[i][0]
                    self.point[i][1]-=p[i][1]
                    self.point[i][2]-=p[i][2]
                self.run()
            if self.move_timeout!=0:
                self.move_count+=time.time()-self.move_timeout
                self.move_timeout=time.time()
        else:
            self.stop()
            self.move_timeout=time.time()
    def upAndDown(self,var):
        self.height=var+99
        self.changeCoordinates('height',0,self.height,0,0,self.height,0)
    def beforeAndAfter(self,var):
        self.changeCoordinates('horizon',var,self.height,0,var,self.height,0)
    def attitude(self,r,p,y):
        r=self.map(int(r),-20,20,-10,10)
        p=self.map(int(p),-20,20,-10,10)
        y=self.map(int(y),-20,20,-10,10)
        pos=self.postureBalance(r,p,y,0)
        self.changeCoordinates('Attitude Angle',pos=pos)
    def IMU6050(self):
        self.balance_flag=True
        self.order=['','','','','']
        pos=self.postureBalance(0,0,0)
        self.changeCoordinates('Attitude Angle',pos=pos)
        time.sleep(2)
        self.imu.Error_value_accel_data,self.imu.Error_value_gyro_data=self.imu.average_filter()
        time.sleep(1)
        while True:
            self.move_count+=time.time()-self.move_timeout
            self.move_timeout=time.time()
            r,p,y=self.imu.imuUpdate()
            r=self.pid.PID_compute(r)
            p=self.pid.PID_compute(p)
            pos=self.postureBalance(r,p,0)
            self.changeCoordinates('Attitude Angle',pos=pos)
            if  (self.order[0]==cmd.CMD_BALANCE and self.order[1]=='0')or(self.balance_flag==True and self.order[0]!='')or(self.move_count>180):
                Thread_conditiona=threading.Thread(target=self.condition)
                Thread_conditiona.start()
                self.balance_flag==False
                break
    def postureBalance(self,r,p,y,h=1):
        b = 76
        w = 76
        l = 136
        if h!=0:
            h=self.height
        pos = np.mat([0.0,  0.0,  h ]).T 
        rpy = np.array([r,  p,  y]) * math.pi / 180 
        R, P, Y = rpy[0], rpy[1], rpy[2]
        rotx = np.mat([[ 1,       0,            0          ],
                     [ 0,       math.cos(R), -math.sin(R)],
                     [ 0,       math.sin(R),  math.cos(R)]])
        roty = np.mat([[ math.cos(P),  0,      -math.sin(P)],
                     [ 0,            1,       0          ],
                     [ math.sin(P),  0,       math.cos(P)]]) 
        rotz = np.mat([[ math.cos(Y), -math.sin(Y),  0     ],
                     [ math.sin(Y),  math.cos(Y),  0     ],
                     [ 0,            0,            1     ]])
        rot_mat = rotx * roty * rotz
        body_struc = np.mat([[ l / 2,  b / 2,  0],
                           [ l / 2, -b / 2,    0],
                           [-l / 2,  b / 2,    0],
                           [-l / 2, -b / 2,    0]]).T
        footpoint_struc = np.mat([[(l / 2),  (w / 2)+10,  self.height-h],
                                [ (l / 2), (-w / 2)-10,    self.height-h],
                                [(-l / 2),  (w / 2)+10,    self.height-h],
                                [(-l / 2), (-w / 2)-10,    self.height-h]]).T
        AB = np.mat(np.zeros((3, 4)))
        for i in range(4):
            AB[:, i] = pos + rot_mat * footpoint_struc[:, i] - body_struc[:, i]
        return (AB)
    
control = Control()


def main(args=None):
    rclpy.init(args=args)

    node_ = DogCommands()

    rclpy.spin(node_)
    DogCommands.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
